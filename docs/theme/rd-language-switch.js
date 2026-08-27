/* One link, in the menu bar, to the same page in the other language.
 *
 * The two sites are built from the same headers and the same set of Markdown
 * files, so a page has the same file name under /en/ and under /zh/. Swapping
 * the one path segment therefore lands on the page the reader was already on,
 * rather than dropping them back at the front door -- which is what makes the
 * switch usable from deep inside the API reference.
 *
 * The current language comes from the path rather than from a build-time
 * substitution, so both sites ship this file byte for byte identical. */
(function () {
    "use strict";

    var LANGS = {
        en: { other: "zh", label: "中文", title: "切换到中文" },
        zh: { other: "en", label: "English", title: "Read this page in English" }
    };

    /* Last path segment that names a language wins, so a project served from
     * a subdirectory called "en" for unrelated reasons does not confuse this. */
    function locate(parts) {
        for (var i = parts.length - 1; i >= 0; --i) {
            if (LANGS[parts[i]]) return i;
        }
        return -1;
    }

    function counterpart() {
        var parts = window.location.pathname.split("/");
        var at = locate(parts);
        if (at < 0) return null;
        var lang = LANGS[parts[at]];
        parts[at] = lang.other;
        return { href: parts.join("/") + window.location.hash, lang: lang };
    }

    function insert() {
        var target = counterpart();
        if (!target) return;

        var link = document.createElement("a");
        link.href = target.href;
        link.textContent = target.lang.label;
        link.title = target.lang.title;
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
