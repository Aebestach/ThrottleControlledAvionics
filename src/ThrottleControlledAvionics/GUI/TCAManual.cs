//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Collections.Generic;
using UnityEngine;
using AT_Utils;
using AT_Utils.UI;

namespace ThrottleControlledAvionics
{
    [KSPAddon(KSPAddon.Startup.AllGameScenes, false)]
    public class TCAManual : AddonWindowBase<TCAManual>
    {
        static MDSection Manual { get { return Globals.Instance.Manual; } }
        static MDSection last_manual;
        static bool show_status;
        static MDSection current_section;
        static string current_text = "";
        static Vector2 sections_scroll;
        static Vector2 content_scroll;
        static List<TCAPart> parts;

        public TCAManual() { width = 800; height = 600; }

        public override void Awake()
        {
            base.Awake();
            GameEvents.onLevelWasLoaded.Add(onSceneChange);
        }

        void onSceneChange(GameScenes scene)
        {
            parts = TCAModulesDatabase.GetPurchasedParts();
        }

        void Update()
        {
            if(Manual == null) return;
            if(Manual != last_manual)
            {
                last_manual = Manual;
                current_section = null;
            }
            if(WindowEnabled)
            {
                if(current_section == null)
                    change_section(Manual.NoText && Manual.Subsections.Count > 0 ?
                                   Manual.Subsections[0] : Manual);
            }
        }

        void change_section(MDSection sec)
        {
            show_status = false;
            current_section = sec;
            current_text = sec.Text;
        }

        static void PartsInfo()
        {
            if(parts == null) return;
            if(parts.Count == 0)
            {
                GUILayout.Label(Loc.T("NoModulesInstalled", "No modules installed."));
                return;
            }
            GUILayout.BeginVertical(Styles.white);
            for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
            {
                var part = parts[i];
                GUILayout.BeginHorizontal();
                GUILayout.Label(part.Title);
                GUILayout.FlexibleSpace();
                if(part.Active) GUILayout.Label(Loc.T("Available", "Available"), Styles.enabled);
                else GUILayout.Label(Loc.Content("DependenciesUnsatisfied", "Dependencies Unsatisfied",
                                                    "DependenciesUnsatisfiedTip", "Consult R&D tree to see what modules are required for this one to work."),
                                     Styles.danger);
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
        }

        public static void ShowStatus()
        {
            if(HighLogic.CurrentGame == null) return;
            show_status = true;
            ShowInstance(true);
        }

        void DrawMainWindow(int windowID)
        {
            GUILayout.BeginVertical();
            sections_scroll = GUILayout.BeginScrollView(sections_scroll, GUILayout.Height(45));
            GUILayout.BeginHorizontal();
            if(HighLogic.CurrentGame != null &&
               GUILayout.Button(Loc.T("Status", "Status"), show_status ?
                                Styles.open_button : Styles.normal_button,
                                GUILayout.ExpandWidth(false)))
                show_status = true;
            for(int i = 0, count = Manual.Subsections.Count; i < count; i++)
            {
                var ss = Manual.Subsections[i];
                if(GUILayout.Button(ss.Title, (!show_status && current_section == ss) ?
                                    Styles.good_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
                    change_section(ss);
            }
            GUILayout.EndHorizontal();
            GUILayout.EndScrollView();
            content_scroll = GUILayout.BeginScrollView(content_scroll, Styles.white_on_black, GUILayout.ExpandHeight(true));
            if(show_status)
            {
                GUILayout.BeginVertical();
                GUILayout.Label(Title);
                if(!TCAScenario.ModuleInstalled)
                    GUILayout.Label(Colors.Danger
                                    .Tag(Loc.T("ModuleNotFound", "<size=30>TCA module was not found in any of the loaded parts.</size>")) + 
                                    "\n\n" + Loc.T("ModuleNotFoundHint", "This probably means you're using an old version of <b>ModuleManager</b> or haven't installed it yet. ") +
                                    Colors.Warning
                                    .Tag(Loc.T("ModuleManagerRequired", "<b>ModuleManager</b> is required")) + Loc.T("ModuleManagerRequiredSuffix", " for TCA to work."),
                                    Styles.rich_label);
                else if(HighLogic.CurrentGame.Mode != Game.Modes.SANDBOX)
                {

                    if(!TCAScenario.HasTCA)
                        GUILayout.Label(Colors.Warning
                                        .Tag(Loc.T("SubsystemNotPurchased", "<size=30>TCA Subsystem is <b>NOT</b> purchased. Get it in R&D first.</size>")),
                                        Styles.rich_label);
                    else if(HighLogic.LoadedSceneIsFlight)
                    {
                        GUILayout.Label(Colors.Good
                                        .Tag(Loc.T("SubsystemPurchased", "TCA Subsystem is purchased.")) + "\n" +
                                        Loc.T("SubsystemPurchasedFlight", "To see TCA modules installed on the current vessel go to <b>Advanced</b> tab."),
                                        Styles.rich_label);
                    }
                    else
                    {
                        GUILayout.Label(Colors.Good
                                        .Tag(Loc.T("SubsystemPurchased", "TCA Subsystem is purchased.")) + 
                                        "\n" + Loc.T("AvailableTCAModules", "Available TCA modules:"),
                                        Styles.rich_label);
                        PartsInfo();
                    }
                }
                else GUILayout.Label(Loc.T("SandboxGame", "<b>Sandbox Game:</b>\n") +
                                     Colors.Good
                                     .Tag(Loc.T("SandboxFunctional", "TCA should be fully functional")) +
                                     Loc.T("SandboxFunctionalSuffix", " on all vessels with some engines/RCS and a command module (cockpit, probe core, etc)."),
                                     Styles.rich_label);
                GUILayout.EndVertical();
            }
            else GUILayout.Label(current_text, Styles.rich_label, GUILayout.MaxWidth(width));
            GUILayout.EndScrollView();
            if(GUILayout.Button(Loc.T("Close", "Close"))) Show(false);
            GUILayout.EndVertical();
            TooltipsAndDragWindow();
        }

        protected override bool can_draw() { return Manual != null; }
        protected override void draw_gui()
        {
            LockControls();
            WindowPos =
                GUILayout.Window(GetInstanceID(),
                                 WindowPos,
                                 DrawMainWindow,
                                 Manual != null && !Manual.NoTitle
                                     ? Manual.Title
                                     : Loc.T("ManualWindowTitle", "TCA Manual"),
                                 GUILayout.Width(width),
                                 GUILayout.Height(height)).clampToScreen();
        }
    }
}

