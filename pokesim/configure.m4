dnl -*- Autoconf -*-

AC_DEFUN([SST_pokesim_CONFIG],[

  AC_ARG_WITH([harplib],
    [AS_HELP_STRING([--with-harplib@<:@=DIR@:>@],
      [Use harplib installed at optionally-specified prefix DIR])])

  sst_check_harplib_happy="yes"

  AS_IF([test "$with_harplib" = "no"], [sst_check_harplib_happy="no"])

  CPPFLAGS_saved="$CPPFLAGS"
  CXXFLAGS_saved="$CXXFLAGS"
  LDFLAGS_saved="$LDFLAGS"
  LIBS_saved="$LIBS"

  AS_IF([test ! -z "$with_harplib" -a "$with_harplib" != "yes"],
    [HARPLIB_CPPFLAGS="-I$with_harplib/include -std=c++11"
     CPPFLAGS="$HARPLIB_CPPFLAGS $CPPFLAGS"
     HARPLIB_CXXFLAGS="-std=c++11"
     CXXFLAGS="$HARPLIB_CXXFLAGS $CXXFLAGS"
     HARPLIB_LDFLAGS="-L$with_harplib/lib"
     LDFLAGS="$HARPLIB_LDFLAGS $LDFLAGS"
     HARPLIB_LIBDIR="$with_harplib/lib"],
    [HARPLIB_CPPFLAGS=
     HARPLIB_CXXFLAGS=
     HARPLIB_LDFLAGS=])

  AC_LANG_PUSH(C++)
  AC_CHECK_HEADERS([harp/core.h], [], [sst_check_harplib_happy="no"])
  AC_CHECK_LIB([harplib], [harplib_present],
    [HARPLIB_LIBS="-lharplib"], [sst_check_harplib_happy="no"])
  AC_LANG_POP(C++)

  CPPFLAGS="$CPPFLAGS_saved"
  CXXFLAGS="$CXXFLAGS_saved"
  LDFLAGS="$LDFLAGS_saved"
  LIBS="$LIBS_saved"

  AC_SUBST([HARPLIB_CPPFLAGS])
  AC_SUBST([HARPLIB_CXXFLGAS])
  AC_SUBST([HARPLIB_LDFLAGS])
  AC_SUBST([HARPLIB_LIBS])
  AM_CONDITIONAL([HAVE_HARPLIB], [test "$sst_check_harplib_happy" = "yes"])
  AS_IF([test "$sst_check_harplib_happy" = "yes"],
        [AC_DEFINE([HAVE_HARPLIB], [1], [Set to 1 if harplib was found])])
  AC_DEFINE_UNQUOTED([HARPLIB_LIBDIR], ["$HARPLIB_LIBDIR"], [Path to harplib library])



  AS_IF([test "$sst_check_harplib_happy" = "yes"], [$1], [$2])
])
