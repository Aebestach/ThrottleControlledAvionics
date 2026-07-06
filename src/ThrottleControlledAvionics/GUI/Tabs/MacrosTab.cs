//   MacrosTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class MacrosTab : ControlTab
    {
        public MacrosTab(ModuleTCA tca) : base(tca) {}

        #pragma warning disable 169
        MacroProcessor MPC;
        #pragma warning restore 169

        bool selecting_macro;
        public override void Draw()
        {
            GUILayout.BeginHorizontal();
            if(CFG.SelectedMacro != null && CFG.MacroIsActive)
            {
                GUILayout.Label(Loc.Content("MacroExecuting", Loc.F("MacroLabel", "Macro: <<1>>", CFG.SelectedMacro.Title), "MacroExecutingTip", "The macro is executing..."), 
                                Styles.warning, GUILayout.ExpandWidth(true));
                CFG.MacroIsActive &= !GUILayout.Button(Loc.T("Pause", "Pause"), Styles.enabled_button, GUILayout.Width(70));
                if(GUILayout.Button(Loc.T("Stop", "Stop"), Styles.danger_button, GUILayout.ExpandWidth(false))) 
                    CFG.StopMacro();
                GUILayout.Label(Loc.T("Edit", "Edit"), Styles.inactive_button, GUILayout.ExpandWidth(false));
            }
            else if(CFG.SelectedMacro != null)
            {
                if(GUILayout.Button(Loc.Content("MacroSelect", Loc.F("MacroLabel", "Macro: <<1>>", CFG.SelectedMacro.Title), "MacroSelectTip", "Select a macro from databases"), 
                                    Styles.normal_button, GUILayout.ExpandWidth(true))) 
                    selecting_macro = !selecting_macro;
                CFG.MacroIsActive |= GUILayout.Button(CFG.SelectedMacro.Active? Loc.T("Resume", "Resume") : Loc.T("Execute", "Execute"), 
                                                      Styles.active_button, GUILayout.Width(70));
                if(GUILayout.Button(Loc.T("Stop", "Stop"), CFG.SelectedMacro.Active? 
                                    Styles.danger_button : Styles.inactive_button, GUILayout.ExpandWidth(false))) 
                    CFG.SelectedMacro.Rewind();
                if(GUILayout.Button(Loc.T("Edit", "Edit"), Styles.active_button, GUILayout.ExpandWidth(false)))
                    TCAMacroEditor.Edit(CFG);
            }
            else 
            {
                if(GUILayout.Button(Loc.T("SelectMacro", "Select Macro"), Styles.normal_button, GUILayout.ExpandWidth(true))) 
                    selecting_macro = !selecting_macro;
                if(GUILayout.Button(Loc.T("NewMacro", "New Macro"), Styles.open_button, GUILayout.ExpandWidth(false)))
                    TCAMacroEditor.Edit(CFG);
            }
            GUILayout.EndHorizontal();
            if(selecting_macro)
            {
                TCAMacro macro = null;
                if(TCAMacroEditor.DrawMacroSelector(CFG, out macro)) 
                {
                    if(macro != null) 
                    {
                        CFG.SelectedMacro = macro.GetCopy() as TCAMacro;
                        CFG.MacroIsActive = false;
                    }
                    selecting_macro = false;
                }
            }
        }
    }
}