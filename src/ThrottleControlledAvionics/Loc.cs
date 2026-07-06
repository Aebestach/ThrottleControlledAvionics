using KSP.Localization;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    /// <summary>
    /// Localization helper for TCA. Keys live in GameData/ThrottleControlledAvionics/Localization/*.cfg
    /// with the #LOC_TCA_ prefix.
    /// </summary>
    public static class Loc
    {
        public const string TagPrefix = "#LOC_TCA_";

        public static string Tag(string key) => TagPrefix + key;

        /// <summary>Resolve a localization key; falls back to English default.</summary>
        public static string T(string key, string fallback)
        {
            var tag = Tag(key);
            return Localizer.TryGetStringByTag(tag, out var result) ? result : fallback;
        }

        /// <summary>Resolve a full localization tag (e.g. #LOC_TCA_Foo); falls back to English default.</summary>
        public static string TTag(string tag, string fallback)
        {
            return Localizer.TryGetStringByTag(tag, out var result) ? result : fallback;
        }

        /// <summary>Format a localized string with &lt;&lt;1&gt;&gt;, &lt;&lt;2&gt;&gt; placeholders.</summary>
        public static string F(string key, string fallback, params object[] args)
        {
            var tag = Tag(key);
            if(Localizer.TryGetStringByTag(tag, out var template))
                return Localizer.Format(template, args);
            return Localizer.Format(fallback, args);
        }

        public static string FTag(string tag, string fallback, params object[] args)
        {
            if(Localizer.TryGetStringByTag(tag, out var template))
                return Localizer.Format(template, args);
            return Localizer.Format(fallback, args);
        }

        public static GUIContent Content(string key, string text, string tooltipKey = null, string tooltip = null)
        {
            var label = T(key, text);
            var tip = tooltipKey != null ? T(tooltipKey, tooltip ?? "") : (tooltip ?? "");
            return string.IsNullOrEmpty(tip) ? new GUIContent(label) : new GUIContent(label, tip);
        }

        /// <summary>Resolve a #LOC_TCA_ tag or a short key with English fallback.</summary>
        public static string Resolve(string keyOrTag, string fallback)
        {
            if(string.IsNullOrEmpty(keyOrTag)) return fallback ?? "";
            return keyOrTag.StartsWith(TagPrefix) ? TTag(keyOrTag, fallback) : T(keyOrTag, fallback);
        }
    }
}
