/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
 * Version: MPL 1.1 / GPLv3+ / LGPLv3+
 *
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License or as specified alternatively below. You may obtain a copy of
 * the License at http: *www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 * for the specific language governing rights and limitations under the
 * License.
 *
 * Major Contributor(s):
 * Copyright (C) 2011 Tor Lillqvist <tml@iki.fi> (initial developer)
 * Copyright (C) 2011 SUSE Linux http://suse.com (initial developer's employer)
 *
 * All Rights Reserved.
 *
 * For minor contributions see the git repository.
 *
 * Alternatively, the contents of this file may be used under the terms of
 * either the GNU General Public License Version 3 or later (the "GPLv3+"), or
 * the GNU Lesser General Public License Version 3 or later (the "LGPLv3+"),
 * in which case the provisions of the GPLv3+ or the LGPLv3+ are applicable
 * instead of those above.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <sys/mman.h>

#include <linux/elf.h>

#include "inc/android-dl.h"
#include "common.h"

/* Taken from bionic/linker/liner.cpp */
static const char* const kDefaultLdPaths[] = {
#if defined(__LP64__)
  "/vendor/lib64",
  "/system/lib64",
#else
  "/vendor/lib",
  "/system/lib",
#endif
  NULL
};

/* The library paths as a double-null terminated string */
char *library_locations;

static char last_error[1024] = {0};

extern "C" {

void set_error(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    vsnprintf(last_error, sizeof(last_error), format, args);
    __android_log_write(ANDROID_LOG_ERROR, LOG_TAG, last_error);

    va_end(args);
}

#define SET_ERROR(format, ...) set_error("%s: " format, __FUNCTION__, ##__VA_ARGS__)

static char *
read_section(int fd,
             Elf32_Shdr *shdr)
{
    char *result = (char*)malloc(shdr->sh_size);
    if (lseek(fd, shdr->sh_offset, SEEK_SET) < 0) {
        close(fd);
        free(result);
        return NULL;
    }
    if (read(fd, result, shdr->sh_size) < (int) shdr->sh_size) {
        close(fd);
        free(result);
        return NULL;
    }

    return result;
}

__attribute__ ((visibility("default")))
char **
android_dlneeds(const char *library)
{
    int i, fd;
    int n_needed;
    char **result;
    char *shstrtab;
    char *dynstr = NULL;
    Elf32_Ehdr hdr;
    Elf32_Shdr shdr;
    Elf32_Dyn dyn;

    /* Open library and read ELF header */

    fd = open(library, O_RDONLY);

    if (fd == -1) {
        SET_ERROR("Could not open library %s: %s", library, strerror(errno));
        return NULL;
    }

    if (read(fd, &hdr, sizeof(hdr)) < (int) sizeof(hdr)) {
        SET_ERROR("Could not read ELF header of %s", library);
        close(fd);
        return NULL;
    }

    /* Read in .shstrtab */

    if (lseek(fd, hdr.e_shoff + hdr.e_shstrndx * sizeof(shdr), SEEK_SET) < 0) {
        SET_ERROR("Could not seek to .shstrtab section header of %s", library);
        close(fd);
        return NULL;
    }
    if (read(fd, &shdr, sizeof(shdr)) < (int) sizeof(shdr)) {
        SET_ERROR("Could not read section header of %s", library);
        close(fd);
        return NULL;
    }

    shstrtab = read_section(fd, &shdr);
    if (shstrtab == NULL)
        return NULL;

    /* Read section headers, looking for .dynstr section */

    if (lseek(fd, hdr.e_shoff, SEEK_SET) < 0) {
        SET_ERROR("Could not seek to section headers of %s", library);
        close(fd);
        return NULL;
    }
    for (i = 0; i < hdr.e_shnum; i++) {
        if (read(fd, &shdr, sizeof(shdr)) < (int) sizeof(shdr)) {
            SET_ERROR("Could not read section header of %s", library);
            close(fd);
            return NULL;
        }
        if (shdr.sh_type == SHT_STRTAB &&
            strcmp(shstrtab + shdr.sh_name, ".dynstr") == 0) {
            dynstr = read_section(fd, &shdr);
            if (dynstr == NULL) {
                free(shstrtab);
                return NULL;
            }
            break;
        }
    }

    if (i == hdr.e_shnum) {
        SET_ERROR("No .dynstr section in %s", library);
        close(fd);
        return NULL;
    }

    /* Read section headers, looking for .dynamic section */

    if (lseek(fd, hdr.e_shoff, SEEK_SET) < 0) {
        SET_ERROR("Could not seek to section headers of %s", library);
        close(fd);
        return NULL;
    }
    for (i = 0; i < hdr.e_shnum; i++) {
        if (read(fd, &shdr, sizeof(shdr)) < (int) sizeof(shdr)) {
            SET_ERROR("Could not read section header of %s", library);
            close(fd);
            return NULL;
        }
        if (shdr.sh_type == SHT_DYNAMIC) {
            size_t dynoff;

            /* Count number of DT_NEEDED entries */
            n_needed = 0;
            if (lseek(fd, shdr.sh_offset, SEEK_SET) < 0) {
                SET_ERROR("Could not seek to .dynamic section of %s", library);
                close(fd);
                return NULL;
            }
            for (dynoff = 0; dynoff < shdr.sh_size; dynoff += sizeof(dyn)) {
                if (read(fd, &dyn, sizeof(dyn)) < (int) sizeof(dyn)) {
                    SET_ERROR("Could not read .dynamic entry of %s", library);
                    close(fd);
                    return NULL;
                }
                if (dyn.d_tag == DT_NEEDED)
                    n_needed++;
            }

            /* LOGI("Found %d DT_NEEDED libs", n_needed); */

            result = (char**)malloc((n_needed+1) * sizeof(char *));

            n_needed = 0;
            if (lseek(fd, shdr.sh_offset, SEEK_SET) < 0) {
                SET_ERROR("Could not seek to .dynamic section of %s", library);
                close(fd);
                free(result);
                return NULL;
            }
            for (dynoff = 0; dynoff < shdr.sh_size; dynoff += sizeof(dyn)) {
                if (read(fd, &dyn, sizeof(dyn)) < (int) sizeof(dyn)) {
                    SET_ERROR("Could not read .dynamic entry in %s", library);
                    close(fd);
                    free(result);
                    return NULL;
                }
                if (dyn.d_tag == DT_NEEDED) {
                    /* LOGI("needs: %s\n", dynstr + dyn.d_un.d_val); */
                    result[n_needed] = strdup(dynstr + dyn.d_un.d_val);
                    n_needed++;
                }
            }

            close(fd);
            if (dynstr)
                free(dynstr);
            free(shstrtab);
            result[n_needed] = NULL;
            return result;
        }
    }

    SET_ERROR("Could not find .dynamic section in %s", library);
    close(fd);
    return NULL;
}

bool
library_exists(const char *path)
{
    struct stat st;

    return
        stat(path, &st) == 0 &&
        S_ISREG(st.st_mode);
}

char *
get_library_full_path(const char *library)
{
    for (const char *dir = library_locations; *dir != '\0'; dir += strlen(dir) + 1) {
        char *full_path;
        
        if (asprintf(&full_path, "%s/%s", dir, library) == -1)
            continue; // should not normally happen

        if (library_exists(full_path))
            return full_path;

        free(full_path);
    }
    
    // Fall back to the built-in well known paths (like bionic's linker)
    for (const char* const* syspath = kDefaultLdPaths; *syspath != NULL; ++syspath) {
        char *full_path;

        if (asprintf(&full_path, "%s/%s", *syspath, library) == -1)
            continue; // should not normally happen

        if (library_exists(full_path))
            return full_path;

        free(full_path);
    }
    
    return NULL;
}

char *
get_library_full_path_env(const char *library)
{
    const char *llp = getenv("LD_LIBRARY_PATH");
    while (llp != NULL && *llp != '\0') {
        char *full_path;

        // find path separator (or NUL)
        char *sep = strchr(llp, ':');
        int len = (sep != NULL) ? sep - llp: -1;

        if (asprintf(&full_path, "%.*s/%s", len, llp, library) == -1)
            continue; // should not normally happen

        if (library_exists(full_path))
            return full_path;

        free(full_path);

        // advance to next path segment
        if (sep == NULL) break;
        llp = sep + 1;
    }

    // Fall back to the built-in well known paths (like bionic's linker)
    for (const char* const* syspath = kDefaultLdPaths; *syspath != NULL; ++syspath) {
        char *full_path;

        if (asprintf(&full_path, "%s/%s", *syspath, library) == -1)
            continue; // should not normally happen

        if (library_exists(full_path))
            return full_path;

        free(full_path);
    }

    return NULL;
}

__attribute__ ((visibility("default")))
void *
android_dlopen(const char *library)
{
    /*
     * We should *not* try to just dlopen() the bare library name
     * first, as the stupid dynamic linker remembers for each library
     * basename if loading it has failed. Thus if you try loading it
     * once, and it fails because of missing needed libraries, and
     * your load those, and then try again, it fails with an
     * infuriating message "failed to load previously" in the log.
     *
     * We *must* first dlopen() all needed libraries, recursively. It
     * shouldn't matter if we dlopen() a library that already is
     * loaded, dlopen() just returns the same value then.
     */

    struct loadedLib {
        const char *name;
        void *handle;
        struct loadedLib *next;
    };
    static struct loadedLib *loaded_libraries = NULL;

    struct loadedLib *rover;
    struct loadedLib *new_loaded_lib;

    void *p;
    char **needed;
    int i;

    struct timeval tv0, tv1, tvdiff;

    rover = loaded_libraries;
    while (rover != NULL &&
           strcmp(rover->name, library) != 0)
        rover = rover->next;

    if (rover != NULL)
        return rover->handle;
    
    /* LOGI("%s(%s)", __FUNCTION__, library); */

    const char *full_name = NULL;
    char *full_name_heap = NULL;
    
    if (library[0] == '/') {
        if (library_exists(library))
            full_name = library;
    } else if (library_locations != NULL) {
        full_name_heap = get_library_full_path(library);
        full_name = full_name_heap;
    } else { // search in LD_LIBRARY_PATH and kDefaultLdPaths
        full_name_heap = get_library_full_path_env(library);
        full_name = full_name_heap;
    }

    if (full_name == NULL) {
        SET_ERROR("Library %s not found", library);
        return NULL;
    }

    needed = android_dlneeds(full_name);
    if (needed == NULL) {
        free(full_name_heap);
        return NULL;
    }

    for (i = 0; needed[i] != NULL; i++) {
        if (android_dlopen(needed[i]) == NULL) {
            free_ptrarray((void **) needed);
            free(full_name_heap);
            return NULL;
        }
    }
    free_ptrarray((void **) needed);

    gettimeofday(&tv0, NULL);
    p = dlopen(full_name, RTLD_LOCAL);
    gettimeofday(&tv1, NULL);
    timersub(&tv1, &tv0, &tvdiff);
    LOGI("dlopen(%s) = %p, %ld.%03lds",
         full_name, p,
         (long) tvdiff.tv_sec, (long) tvdiff.tv_usec / 1000);
    if (p == NULL)
        SET_ERROR("Error from dlopen(%s): %s", full_name, dlerror());
    free(full_name_heap);

    new_loaded_lib = (struct loadedLib*)malloc(sizeof(*new_loaded_lib));
    new_loaded_lib->name = strdup(library);
    new_loaded_lib->handle = p;

    new_loaded_lib->next = loaded_libraries;
    loaded_libraries = new_loaded_lib;

    return p;
}

__attribute__ ((visibility("default")))
void *
android_dlsym(void *handle,
         const char *symbol)
{
    void *p = dlsym(handle, symbol);
    if (p == NULL)
        set_error("%s(%p,%s): %s", __FUNCTION__, handle, symbol, dlerror());
    return p;
}

__attribute__ ((visibility("default")))
int
android_dladdr(void *addr,
          Dl_info *info)
{
    FILE *maps;
    char line[200];
    int result;
    int found;

    result = dladdr(addr, info);
    if (result == 0) {
        /* LOGI("dladdr(%p) = 0", addr); */
        return 0;
    }

    maps = fopen("/proc/self/maps", "r");
    if (maps == NULL) {
        SET_ERROR("Could not open /proc/self/maps: %s", strerror(errno));
        return 0;
    }

    found = 0;
    while (fgets(line, sizeof(line), maps) != NULL &&
           line[strlen(line)-1] == '\n') {
        void *lo, *hi;
        char file[sizeof(line)];
        file[0] = '\0';
        if (sscanf(line, "%x-%x %*s %*x %*x:%*x %*d %[^\n]", (unsigned *) &lo, (unsigned *) &hi, file) == 3) {
            /* LOGI("got %p-%p: %s", lo, hi, file); */
            if (addr >= lo && addr < hi) {
                if (info->dli_fbase != lo) {
                    SET_ERROR("Base for %s in /proc/self/maps %p doesn't match what dladdr() said", file, lo);
                    fclose(maps);
                    return 0;
                }
                /* LOGI("dladdr(%p) = { %s:%p, %s:%p }: %s",
                     addr,
                     info->dli_fname, info->dli_fbase,
                     info->dli_sname ? info->dli_sname : "(none)", info->dli_saddr,
                     file); */
                info->dli_fname = strdup(file);
                found = 1;
                break;
            }
        }
    }
    if (!found)
        SET_ERROR("Did not find %p in /proc/self/maps", addr);
    fclose(maps);

    return result;
}

__attribute__ ((visibility("default")))
int
android_dlclose(void *handle)
{
    /* As we don't know when the reference count for a dlopened shared
     * object drops to zero, we wouldn't know when to remove it from
     * our list, so we can't call dlclose().
     */
    LOGI("%s(%p)", __FUNCTION__, handle);

    return 0;
}

__attribute__ ((visibility("default")))
const char *
android_dl_get_last_error()
{
    return last_error;
}

} // extern "C"

/* vim:set shiftwidth=4 softtabstop=4 expandtab: */
