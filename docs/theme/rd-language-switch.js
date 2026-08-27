/* One link, in the menu bar, to the same page in the other language.
 *
 * English is the site root and Chinese sits one directory below it at zh/.
 * Both are built from the same headers and the same set of Markdown files, so
 * a page has the same file name on either side: switching therefore lands on
 * the page the reader was already on rather than dropping them at the front
 * door, which is what makes it usable from inside the API reference.
 *
 * The site root comes from Doxygen's own $relpath^, substituted into the
 * header as RD_DOCS_RELPATH. Counting path segments would work too until the
 * day the site moves under a directory that happens to be called zh. */
(function () {
    "use strict";

    var LABEL = { toZh: { text: "中文",    title: "切换到中文" },
                  toEn: { text: "English", title: "Read this page in English" } };

    function counterpart() {
        var here = window.location.href;
        /* Every page Doxygen writes sits flat in its output directory, so the
         * directory of the current page is that language's root. */
        var base = new URL(window.RD_DOCS_RELPATH || "./", here);
        if (here.indexOf(base.href) !== 0) return null;

        var page = here.slice(base.href.length);        /* name.html#anchor */
        var zh   = /\/zh\/$/.test(base.pathname);
        return { href: new URL(zh ? "../" : "zh/", base).href + page,
                 label: zh ? LABEL.toEn : LABEL.toZh };
    }

    function insert() {
        var target = counterpart();
        if (!target) return;

        var link = document.createElement("a");
        link.href = target.href;
        link.textContent = target.label.text;
        link.title = target.label.title;
        link.className = "rd-language-switch";

        /* The menu bar when Doxygen draws one, and the title block when it
         * does not -- a search-less or index-less build still gets the link. */
        var menu = document.getElementById("main-menu");
        if (menu) {
            var item = document.createElement("li");
            item.className = "rd-language-item";
            item.appendChild(link);
            menu.appendChild(item);
            return;
        }
        var title = document.getElementById("titlearea");
        if (title) title.appendChild(link);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", insert);
    } else {
        insert();
    }
})();
