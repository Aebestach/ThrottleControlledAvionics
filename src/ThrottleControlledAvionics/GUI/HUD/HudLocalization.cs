//   HudLocalization.cs
//
//  Copyright (c) 2026 Allis Tauri

using AT_Utils.UI;
using UnityEngine;
using UnityEngine.UI;

namespace ThrottleControlledAvionics
{
    static class HudLocalization
    {
        public static void SetTooltip(TooltipTrigger tooltip, string tag, string defaultText)
        {
            if(tooltip == null)
                return;
            tooltip.SetText(string.IsNullOrEmpty(tag) ? defaultText : Loc.T(tag, defaultText));
        }

        public static void SetToggleTooltip(Toggle toggle, string tag, string defaultText)
        {
            if(toggle == null)
                return;
            SetTooltip(toggle.GetComponent<TooltipTrigger>(), tag, defaultText);
        }

        public static void SetToggleLabel(Toggle toggle, string tag, string defaultText)
        {
            if(toggle == null)
                return;
            var label = toggle.GetComponentInChildren<Text>();
            if(label != null)
                label.text = Loc.T(tag, defaultText);
        }

        public static void SetIndicatorTooltip(Indicator indicator, string tag, string defaultText)
        {
            if(indicator?.tooltip != null)
                indicator.tooltip.SetText(Loc.T(tag, defaultText));
        }
    }
}
