#!/usr/bin/env sh
# Build both documentation sites into docs/.site/{en,zh}.
#
# The CI workflow runs this same script, so what lands on GitHub Pages is what
# you get locally. Needs doxygen, graphviz and git on PATH; everything else it
# fetches or generates.
#
#     docs/build.sh            build both sites
#     docs/build.sh en         build one
#     docs/build.sh --strict   fail if Doxygen warns about anything
#
# --strict is what CI runs. A doc comment that drifts away from the thing it
# documents -- a renamed parameter, a @ref to a page that moved, a comment
# separated from its declaration by an edit above it -- warns and nothing else,
# so without this the site quietly rots one commit at a time.
set -eu

STRICT=0
LANGS=""
for arg in "$@"; do
    case "$arg" in
        --strict) STRICT=1 ;;
        en|zh)    LANGS="$LANGS $arg" ;;
        *)        echo "usage: docs/build.sh [--strict] [en|zh]" >&2; exit 2 ;;
    esac
done

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"

THEME_TAG=v2.4.2
THEME_REPO=https://github.com/jothepro/doxygen-awesome-css.git
THEME_DIR=docs/.theme
BUILD_DIR=docs/.build
SITE_DIR=docs/.site

# --- The version the site announces --------------------------------------
# Read from the header rather than kept here, so that a release bumps one
# number in one place and the site follows.
RD_VERSION=$(sed -n 's/^ \* @version \(.*\)$/\1/p' RobotDynamics/robot_dynamics.h)
if [ -z "$RD_VERSION" ]; then
    echo "docs/build.sh: no @version in RobotDynamics/robot_dynamics.h" >&2
    exit 1
fi
export RD_VERSION

# --- Theme ----------------------------------------------------------------
# Pinned to a tag: a theme that moves under the site would change every page
# on an unrelated commit, and the diff would be invisible in review.
if [ ! -f "$THEME_DIR/doxygen-awesome.css" ]; then
    echo "fetching doxygen-awesome-css $THEME_TAG"
    rm -rf "$THEME_DIR"
    git -c advice.detachedHead=false clone --quiet --depth 1 \
        --branch "$THEME_TAG" "$THEME_REPO" "$THEME_DIR"
    rm -rf "$THEME_DIR/.git"
fi

# --- Header ---------------------------------------------------------------
# Generated from the installed Doxygen rather than checked in, so the template
# always matches the binary that will consume it. A header checked in at one
# Doxygen version and used by another loses whatever that version added to it,
# quietly and with no warning worth reading.
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
doxygen -w html "$BUILD_DIR/header.html" "$BUILD_DIR/footer.html" \
                "$BUILD_DIR/style.css" docs/Doxyfile.en >/dev/null

# Written to a file rather than passed in with awk -v: a BSD awk rejects a
# newline inside a -v assignment, and the failure is a header that silently
# loses every script tag.
cat > "$BUILD_DIR/head-snippet.html" <<'SNIPPET'
<script type="text/javascript" src="$relpath^doxygen-awesome-darkmode-toggle.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-fragment-copy-button.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-paragraph-link.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-interactive-toc.js"></script>
<script type="text/javascript" src="$relpath^rd-language-switch.js"></script>
<script type="text/javascript">
    DoxygenAwesomeDarkModeToggle.init()
    DoxygenAwesomeFragmentCopyButton.init()
    DoxygenAwesomeParagraphLink.init()
    DoxygenAwesomeInteractiveToc.init()
</script>
SNIPPET

awk -v ins="$BUILD_DIR/head-snippet.html" '
    /<\/head>/ && !done {
        while ((getline line < ins) > 0) print line
        close(ins)
        done = 1
    }
    { print }
' "$BUILD_DIR/header.html" > "$BUILD_DIR/header.patched" \
    && mv "$BUILD_DIR/header.patched" "$BUILD_DIR/header.html"

grep -q 'rd-language-switch.js' "$BUILD_DIR/header.html" || {
    echo "docs/build.sh: the generated header has no </head> to patch" >&2
    exit 1
}

# --- Sites ----------------------------------------------------------------
build_one() {
    echo "doxygen: $1"
    rm -rf "$SITE_DIR/$1"
    doxygen "docs/Doxyfile.$1" 2> "$BUILD_DIR/warnings.$1"
    if [ -s "$BUILD_DIR/warnings.$1" ]; then
        cat "$BUILD_DIR/warnings.$1" >&2
        if [ "$STRICT" = 1 ]; then
            echo "docs/build.sh: doxygen warned, and --strict is on" >&2
            exit 1
        fi
    fi
}

mkdir -p "$SITE_DIR"
if [ -n "$LANGS" ]; then
    for l in $LANGS; do build_one "$l"; done
else
    rm -rf "$SITE_DIR"; mkdir -p "$SITE_DIR"
    build_one en
    build_one zh
    cp docs/theme/landing.html "$SITE_DIR/index.html"
    # GitHub Pages runs Jekyll over what it is given, and Jekyll drops every
    # directory whose name begins with an underscore. Doxygen makes none today;
    # its search index and future output might, and the failure would be a 404
    # on one page rather than a broken build.
    : > "$SITE_DIR/.nojekyll"
fi

echo "built $SITE_DIR"
