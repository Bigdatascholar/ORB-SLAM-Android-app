#include <jni.h>
#include <stdlib.h>
#include <stdio.h>

#include "android-dl.h"
#include "common.h"

extern char *library_locations; // defined in android-dl.cpp

#define METHOD_NAME(class, method) Java_com_github_ikonst_android_1dl_##class##_##method

jclass
	g_cls_String,
	g_cls_IllegalArgumentException,
	g_cls_RuntimeException,
	g_cls_UnsatisfiedLinkError;

jclass FindGlobalClass(JNIEnv *env, const char *className)
{
	jclass cls = env->FindClass(className);
	if (cls == NULL)
		return NULL;
	jclass globalClass = (jclass)env->NewGlobalRef(cls);
	env->DeleteLocalRef(cls);
	return globalClass;
}

/* The dlneeds shared library is always loaded from Java, so
 * this is always called by JNI first.
 */
extern "C"
JNIEXPORT
jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
	(void) reserved;

	// Get environment
	JNIEnv *env;
	if (vm->GetEnv((void**)&env, JNI_VERSION_1_2) != JNI_OK)
	{
		LOGE("Unable to get environment.");
		return JNI_ERR;
	}

	// Globally reference classes
	g_cls_String = FindGlobalClass(env, "java/lang/String");
	g_cls_RuntimeException = FindGlobalClass(env, "java/lang/RuntimeException");
	g_cls_IllegalArgumentException = FindGlobalClass(env, "java/lang/IllegalArgumentException");
	g_cls_UnsatisfiedLinkError = FindGlobalClass(env, "java/lang/UnsatisfiedLinkError");

	return JNI_VERSION_1_2;
}

JNIEXPORT void JNI_OnUnload(JavaVM *vm, __attribute__((unused)) void *reserved)
{
	// Get environment
	JNIEnv *env;
	if(vm->GetEnv((void**)&env, JNI_VERSION_1_2) != JNI_OK)
	{
		LOGE("Unable to get environment.");
		return;
	}

	// Free globally referenced classes
	env->DeleteGlobalRef(g_cls_String);
	env->DeleteGlobalRef(g_cls_RuntimeException);
	env->DeleteGlobalRef(g_cls_IllegalArgumentException);
	env->DeleteGlobalRef(g_cls_UnsatisfiedLinkError);
}

void log_library_locations()
{
    int i = 0;

    // iterate double-NUL terminated string
    for (const char *ll = library_locations; *ll != '\0'; ll += strlen(ll) + 1) {
        LOGI("library_locations[%d] = %s", i, ll);
        ++i;
    }
}

/*
 * Class:     android_dl.AndroidDl
 * Method:    setup
 * Signature: (Ljava/lang/String;[Ljava/lang/String;)Z
 */
extern "C"
JNIEXPORT
jboolean JNICALL METHOD_NAME(AndroidDl, setup)
  (JNIEnv *env, jclass, jstring nativeLibraryDir, jobjectArray ld_library_path)
{
    if ((nativeLibraryDir == NULL) || (ld_library_path == NULL))
    {
        env->ThrowNew(g_cls_IllegalArgumentException, "Invalid arguments.");
        return JNI_FALSE;
    }

    jsize paths_num = env->GetArrayLength(ld_library_path);

    //
    // Calculate the size of the double-NUL terminated string
    //

    jsize size = 1; // final NUL for double-NUL termination

    size += env->GetStringUTFLength(nativeLibraryDir) + 1;

    for (jsize i = 0; i < paths_num; ++i) {
        jstring elem = (jstring)env->GetObjectArrayElement(ld_library_path, i);
        size += env->GetStringUTFLength(elem) + 1;
    }

    //
    // Try to allocate
    //

    library_locations = (char*)malloc(size);
    if (library_locations == NULL) {
        env->ThrowNew(g_cls_RuntimeException, "Cannot allocate library locations.");
        return JNI_FALSE;
    }


    //
    // Copy into library_locations
    //

    char *ll = library_locations;
    int ret;

    // dataDir
    const char *nativeLibraryDirPath = env->GetStringUTFChars(nativeLibraryDir, NULL);
    if (nativeLibraryDirPath == NULL)
    {
        env->ThrowNew(g_cls_RuntimeException, "Cannot get data directory path.");
        return JNI_FALSE;
    }
    ret = sprintf(ll, "%s", nativeLibraryDirPath);
    env->ReleaseStringUTFChars(nativeLibraryDir, nativeLibraryDirPath);
    if (ret < 0) return JNI_FALSE;
    ll += ret + 1;

    // ld_library_path
    for (jsize i = 0; i < paths_num; ++i) {
        jstring elem = (jstring)env->GetObjectArrayElement(ld_library_path, i);
        const char *s = env->GetStringUTFChars(elem, NULL);
        ret = sprintf(ll, "%s", s);
        env->ReleaseStringUTFChars(elem, s);
        if (ret < 0) return JNI_FALSE;
        ll += ret + 1;
    }

    // add final NUL (for double-NUL termination)
    *ll = '\0';

    log_library_locations();

    return JNI_TRUE;
}

/*
 * Class:     android_dl.AndroidDl
 * Method:    dlneeds
 * Signature: (Ljava/lang/String;)[Ljava/lang/String;
 */
extern "C"
JNIEXPORT
jobjectArray JNICALL METHOD_NAME(AndroidDl, dlneeds)
  (JNIEnv *env, jclass, jstring library)
{
	if (library == NULL)
	{
		env->ThrowNew(g_cls_IllegalArgumentException, "Invalid arguments.");
		return 0;
	}

	const char *libName = env->GetStringUTFChars(library, NULL);
	if (libName == NULL)
	{
		env->ThrowNew(g_cls_RuntimeException, "Cannot get library name.");
		return 0;
	}

	char **needed = android_dlneeds(libName);

	env->ReleaseStringUTFChars(library, libName);

	if (needed == NULL)
		return NULL;

	int n_needed = 0;
	while (needed[n_needed] != NULL)
		n_needed++;

	/* Allocate return value */
	jobjectArray result = env->NewObjectArray(n_needed, g_cls_String, NULL);
	if (result == NULL) {
		LOGE("Could not create the String array");
		free_ptrarray((void **) needed);
		return NULL;
	}

	for (n_needed = 0; needed[n_needed] != NULL; n_needed++)
		env->SetObjectArrayElement(result, n_needed, env->NewStringUTF(needed[n_needed]));

	free_ptrarray((void **) needed);

	return result;
}

/*
 * Class:     android_dl.AndroidDl
 * Method:    dlopen
 * Signature: (Ljava/lang/String;)I
 */
extern "C"
JNIEXPORT
jlong JNICALL METHOD_NAME(AndroidDl, dlopen)
  (JNIEnv *env, jclass, jstring library)
{
	if (library == NULL)
	{
		env->ThrowNew(g_cls_IllegalArgumentException, "Invalid arguments.");
		return 0;
	}

	const char *libName = env->GetStringUTFChars(library, NULL);
	if (libName == NULL)
	{
		env->ThrowNew(g_cls_RuntimeException, "Cannot get library name.");
		return 0;
	}
	void *p = android_dlopen (libName);
	env->ReleaseStringUTFChars(library, libName);

	if (p == NULL)
	{
		env->ThrowNew(g_cls_UnsatisfiedLinkError, android_dl_get_last_error());
	}

	return (jlong) p;
}

/*
 * Class:     android_dl.AndroidDl
 * Method:    dlsym
 * Signature: (ILjava/lang/String;)I
 */
extern "C"
JNIEXPORT
jlong JNICALL METHOD_NAME(AndroidDl, dlsym)
  (JNIEnv *env, jclass, jlong handle, jstring symbol)
{
	if ((handle == 0) || (symbol == NULL))
	{
		env->ThrowNew(g_cls_IllegalArgumentException, "Invalid arguments.");
		return 0;
	}

	const char *symName = env->GetStringUTFChars(symbol, NULL);
	if (symName == NULL)
	{
		env->ThrowNew(g_cls_RuntimeException, "Cannot get symbol name.");
		return 0;
	}

	void *p = android_dlsym ((void *) handle, symName);

	env->ReleaseStringUTFChars(symbol, symName);

	if (p == NULL)
	{
		env->ThrowNew(g_cls_UnsatisfiedLinkError, android_dl_get_last_error());
	}

	return (jlong) p;
}

/*
 * Class:     android_dl.AndroidDl
 * Method:    dlcall
 * Signature: (I[Ljava/lang/String;)I
 */
extern "C"
JNIEXPORT
jint JNICALL METHOD_NAME(AndroidDl, dlcall)
  (JNIEnv *env, jclass, jlong function, jobjectArray arguments)
{
	if ((function == 0) || (arguments == NULL))
	{
		env->ThrowNew(g_cls_IllegalArgumentException, "Invalid arguments.");
		return 0;
	}

	int argc = env->GetArrayLength(arguments);
	const char **argv = (const char**)malloc(sizeof(char *) * (argc+1));
	if (argv == NULL)
	{
		env->ThrowNew(g_cls_RuntimeException, "Unable to allocate arguments.");
		return 0;
	}

	for (int i = 0; i < argc; ++i) {
		argv[i] = env->GetStringUTFChars((jstring)env->GetObjectArrayElement(arguments, i), NULL);
	}
	argv[argc] = NULL;

	android_dlcall_func_t fp = (android_dlcall_func_t)function;
	int result = fp(argc, argv);

	for (int i = 0; i < argc; ++i)
		env->ReleaseStringUTFChars((jstring)env->GetObjectArrayElement(arguments, i), argv[i]);

	free(argv);
	return result;
}

/* vim:set shiftwidth=4 softtabstop=4 expandtab: */
