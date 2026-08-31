set -e
set -x

if command -v brew; then
    brew install \
        cmake ninja tcl-tk@8 swig bison flex readline eigen dwarfutils libelf \
        autoconf automake libtool
fi
if command -v apk; then
    apk add curl tcl-dev swig bison flex flex-dev libdwarf-dev elfutils-dev \
        zlib-dev readline-dev eigen-dev automake autoconf libtool
fi
if command -v yum; then
    yum install -y swig flex zlib-devel readline-devel eigen3-devel \
        elfutils-devel elfutils-libelf-devel libdwarf-devel binutils-devel m4 \
        perl-core tcl-devel
fi

NPROC=$(getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu)
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
SUDO="sudo"
fi
    
# Bison
if ! printf '%s\n' '%require "3.8"' '%%' 'start: ;' | bison -o /dev/null /dev/stdin ; then
    WORKDIR=$(mktemp -d)
    (
        cd $WORKDIR
        curl -L --retry 5 --retry-delay 3 \
            https://ftp.gnu.org/gnu/bison/bison-3.8.2.tar.gz | \
            tar --strip-components=1 -xzC .
        ./configure
        make clean
        $SUDO make install -j$NPROC
    )
rm -rf "$WORKDIR"
fi

# CUDD
CUDD_VERSION=3.0.0
WORKDIR=$(mktemp -d)
(
    curl -fL --retry 5 --retry-delay 3 \
    "https://raw.githubusercontent.com/davidkebo/cudd/main/cudd_versions/cudd-${CUDD_VERSION}.tar.gz" \
    | tar -xzC "$WORKDIR"

    cd "$WORKDIR/cudd-${CUDD_VERSION}"
    ./configure
    make -j"$NPROC"
    $SUDO make install
)
rm -rf "$WORKDIR"
