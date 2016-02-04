package com.github.ikonst.android_dl;

/**
 * Fixes Android by allowing loading of libraries and their dependencies.
 *
 * This class exists mostly as a workaround to the Android NDK limitation
 * of being unable to automatically load library dependencies when naively
 * calling @{link System.loadLibrary}.
 */
public class AndroidDl {

	/**
	 * Initializes the library loader.
	 *
	 * @param nativeLibraryDir the native library directory
	 */
	public static boolean initialize(String nativeLibraryDir, String[] ldLibraryPath) {
		System.loadLibrary("android-dl");
		return setup(nativeLibraryDir, ldLibraryPath);
	}

	/**
	 * Initializes the library loader with the system's LD_LIBRARY_PATH.
	 *
	 * Call this method within your main activity and pass
	 *  <code>this.getApplicationInfo().nativeLibraryDir</code> as your native
	 *  library directory.
	 *
	 * @param nativeLibraryDir the native library directory
	 */
	public static boolean initialize(String nativeLibraryDir) {
		String ldLibPath = System.getenv("LD_LIBRARY_PATH");
		String [] path = new String[]{};
		if(ldLibPath != null)
			path =ldLibPath.split(":");
		return initialize(nativeLibraryDir, path);
	}


	/**
	 * Loads a dynamic library for JNI use. Use this method instead of {@link System.loadLibrary}.
	 *
	 * @param libName the name of the library without <code>lib</code> prefix or <code>.so</code> suffix
	 *
	 * @throws UnsatisfiedLinkError if the library could not be loaded.
	 */
	public static void loadLibrary(String libName) {
		String filename = String.format("lib%s.so", libName);
		dlopen(filename);
		System.loadLibrary(libName);
	}

	/**
	 * Internal method to initialize the library loader.
	 */
	private static native boolean setup(String dataDir, String[] ldLibraryPath);

	/**
	 * Gets the names of libraries required by a given library (i.e. the contents of the ELF DT_NEEDED section).
	 *
	 * @param libraryPath the path to the library file (unlike {@link dlopen}, this function does not search the
	 *   library paths)
	 */
	public static native String[] dlneeds(String libraryPath);

	/**
	 * Opens a dynamic library.
	 *
	 * If you plan on using the library for JNI, you should  call {@link loadLibrary} instead.
	 *
	 * @param library the name of the library -- can be either specified in the form of <code>libfoo.so</code> (relative to one of the library directories
	 *  passed to {@link setup}) or as an absolute path
	 * @return a handle to the open library
	 *
	 * @throws UnsatisfiedLinkError if the library could not be loaded.
	 */
	public static native long dlopen(String library);

	/**
	 * Gets a dynamic library symbol.
	 *
	 * @param handle the library handle as previously returned by {@link dlopen}.
	 * @param the name of the symbol
	 * @return an address to the symbol
	 *
	 * @throws UnsatisfiedLinkError if the symbol could not be located.
	 */
	public static native long dlsym(long handle, String symbol);

	/**
	 * Calls a function with a <code>int f(int, const char **)</code> signature.
	 *
	 * @param function the address of a symbol as previously returned by {@link dlsym}
	 * @param arguments the arguments
	 * @return the return value of the function
	 */
	public static native int dlcall(long function, String[] arguments);
}
