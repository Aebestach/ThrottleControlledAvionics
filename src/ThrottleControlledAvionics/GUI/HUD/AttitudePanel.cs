//   AttitudePanel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri

using System.Collections.Generic;
using AT_Utils.UI;
using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class AttitudePanel : ControlPanel<AttitudeUI>
    {
        private static string CueLong(Attitude att)
        {
            switch(att)
            {
            case Attitude.KillRotation: return Loc.T("AttitudeKillRotation", "Kill Rotation");
            case Attitude.HoldAttitude: return Loc.T("AttitudeHoldAttitude", "Hold Attitude");
            case Attitude.ManeuverNode: return Loc.T("AttitudeManeuverNode", "Maneuver Node");
            case Attitude.Prograde: return Loc.T("AttitudePrograde", "Prograde");
            case Attitude.Retrograde: return Loc.T("AttitudeRetrograde", "Retrograde");
            case Attitude.Radial: return Loc.T("AttitudeRadial", "Radial");
            case Attitude.AntiRadial: return Loc.T("AttitudeAntiRadial", "Anti Radial");
            case Attitude.Normal: return Loc.T("AttitudeNormal", "Normal");
            case Attitude.AntiNormal: return Loc.T("AttitudeAntiNormal", "Anti Normal");
            case Attitude.Target: return Loc.T("AttitudeTarget", "To Target");
            case Attitude.AntiTarget: return Loc.T("AttitudeAntiTarget", "From Target");
            case Attitude.RelVel: return Loc.T("AttitudeRelVel", "Relative Velocity");
            case Attitude.AntiRelVel: return Loc.T("AttitudeAntiRelVel", "Against Relative Velocity");
            case Attitude.TargetCorrected: return Loc.T("AttitudeTargetCorrected", "To Target, correcting lateral velocity");
            case Attitude.Custom: return Loc.T("AttitudeCustom", "Attitude is controlled by autopilot");
            default: return "";
            }
        }

        private static string CueShort(Attitude att)
        {
            switch(att)
            {
            case Attitude.KillRotation: return Loc.T("AttitudeKillShort", "Kill");
            case Attitude.HoldAttitude: return Loc.T("AttitudeHoldShort", "Hold");
            case Attitude.ManeuverNode: return Loc.T("AttitudeManeuverShort", "Maneuver");
            case Attitude.Prograde: return Loc.T("AttitudeProgradeShort", "PG");
            case Attitude.Retrograde: return Loc.T("AttitudeRetrogradeShort", "RG");
            case Attitude.Radial: return Loc.T("AttitudeRadialShort", "R+");
            case Attitude.AntiRadial: return Loc.T("AttitudeAntiRadialShort", "R-");
            case Attitude.Normal: return Loc.T("AttitudeNormalShort", "N+");
            case Attitude.AntiNormal: return Loc.T("AttitudeAntiNormalShort", "N-");
            case Attitude.Target: return Loc.T("AttitudeTargetShort", "T+");
            case Attitude.AntiTarget: return Loc.T("AttitudeAntiTargetShort", "T-");
            case Attitude.RelVel: return Loc.T("AttitudeRelVelShort", "rV+");
            case Attitude.AntiRelVel: return Loc.T("AttitudeAntiRelVelShort", "rV-");
            case Attitude.TargetCorrected: return Loc.T("AttitudeTargetCorrectedShort", "T+ rV-");
            case Attitude.Custom: return Loc.T("AttitudeCustomShort", "Auto");
            default: return "";
            }
        }

        private AttitudeControl ATC;

        protected override bool shouldShow => base.shouldShow && ATC != null;

        protected override void init_controller()
        {
            Controller.CurrentCue.CurrentCueSwitch.onClick.AddListener(disableCurrentCue);
            Controller.CuesPanel.Kill.onValueChanged.AddListener(state => onCueChange(Attitude.KillRotation, state));
            Controller.CuesPanel.Hold.onValueChanged.AddListener(state => onCueChange(Attitude.HoldAttitude, state));
            Controller.CuesPanel.Maneuver.onValueChanged.AddListener(state =>
                onCueChange(Attitude.ManeuverNode, state));
            Controller.CuesPanel.PG.onValueChanged.AddListener(state => onCueChange(Attitude.Prograde, state));
            Controller.CuesPanel.RG.onValueChanged.AddListener(state => onCueChange(Attitude.Retrograde, state));
            Controller.CuesPanel.Rp.onValueChanged.AddListener(state => onCueChange(Attitude.Radial, state));
            Controller.CuesPanel.Rm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiRadial, state));
            Controller.CuesPanel.Np.onValueChanged.AddListener(state => onCueChange(Attitude.Normal, state));
            Controller.CuesPanel.Nm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiNormal, state));
            Controller.CuesPanel.Tp.onValueChanged.AddListener(state => onCueChange(Attitude.Target, state));
            Controller.CuesPanel.Tm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiTarget, state));
            Controller.CuesPanel.rVp.onValueChanged.AddListener(state => onCueChange(Attitude.RelVel, state));
            Controller.CuesPanel.rVm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiRelVel, state));
            Controller.CuesPanel.Tp_rVm.onValueChanged.AddListener(
                state => onCueChange(Attitude.TargetCorrected, state));
            base.init_controller();
        }

        private void onCueChange(Attitude cue, bool state)
        {
            if(state)
                CFG.AT.XOnIfNot(cue);
            else
                CFG.AT.XOffIfOn(cue);
        }

        private void disableCurrentCue() => CFG.AT.XOff();

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!IsShown)
                return;
            Controller.SetState(CFG.AT && !VSL.AutopilotDisabled);
            //current cue
            if(CFG.AT)
            {
                Controller.CurrentCue.SetActive(true);
                Controller.CurrentCue.AttitudeError.text = VSL.AutopilotDisabled
                    ? Loc.T("AttitudeUser", "USER")
                    : Loc.F("AttitudeError", "Err: <<1>>°", VSL.Controls.AttitudeError.ToString("F1"));
                Controller.CurrentCue.AttitudeError.color = VSL.Controls.Aligned
                    ? Colors.Enabled
                    : Colors.Neutral;
                Controller.CurrentCue.CurrentCueText.text = CueShort(CFG.AT.state);
                Controller.CurrentCue.CurrentCueTooltip.text = Loc.F("AttitudeCueTooltip", "<<1>>. Click to disable.", CueLong(CFG.AT.state));
            }
            else
                Controller.CurrentCue.SetActive(false);
            //cues
            var controllable = TCA.IsControllable;
            Controller.TSASToggle.SetInteractable(controllable);
            if(!controllable && Controller.TSASToggle.isOn)
                Controller.TSASToggle.isOn = false;
            Controller.CurrentCue.CurrentCueSwitch.SetInteractable(controllable);
            Controller.CuesPanel.SetState(
                CFG.AT[Attitude.KillRotation],
                CFG.AT[Attitude.HoldAttitude],
                CFG.AT[Attitude.ManeuverNode],
                CFG.AT[Attitude.Prograde],
                CFG.AT[Attitude.Retrograde],
                CFG.AT[Attitude.Radial],
                CFG.AT[Attitude.AntiRadial],
                CFG.AT[Attitude.Normal],
                CFG.AT[Attitude.AntiNormal],
                CFG.AT[Attitude.Target],
                CFG.AT[Attitude.AntiTarget],
                CFG.AT[Attitude.RelVel],
                CFG.AT[Attitude.AntiRelVel],
                CFG.AT[Attitude.TargetCorrected]
            );
        }

        #if DEBUG
        protected override void OnRender()
        {
            base.OnRender();
            ATC.DrawDebugLines();
        }
        #endif
    }
}
