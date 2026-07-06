//   VFlightPanel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri

using AT_Utils.UI;
using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class VFlightPanel : ControlPanel<VFlightUI>
    {
        private VerticalSpeedControl VSC;
        private BearingControl BRC;
        private AltitudeControl ALT;
        private ThrottleControl THR;
        private Radar RAD;

        protected override bool shouldShow => base.shouldShow && VSL.OnPlanet && AllModules.Count > 0;

        protected override void init_controller()
        {
            Controller.hoverButton.onValueChanged.RemoveListener(onHover);
            Controller.followTerrainButton.onValueChanged.RemoveListener(onFollowTerrain);
            Controller.autoThrottleButton.onValueChanged.RemoveListener(onAutoThrottle);
            Controller.VSC.onValueChanged.RemoveListener(onVSC);
            Controller.ALT.onValueChanged.RemoveListener(onALT);
            if(ALT != null)
            {
                Controller.ALT.onValueChanged.AddListener(onALT);
                Controller.hoverButton.onValueChanged.AddListener(onHover);
                if(RAD != null)
                    Controller.followTerrainButton.onValueChanged.AddListener(onFollowTerrain);
                else
                    Controller.followTerrainButton.gameObject.SetActive(false);
            }
            else
            {
                Controller.hoverButton.gameObject.SetActive(false);
                Controller.followTerrainButton.gameObject.SetActive(false);
                Controller.ALT.SetActive(false);
            }
            if(THR != null && (ALT != null || VSC != null))
                Controller.autoThrottleButton.onValueChanged.AddListener(onAutoThrottle);
            else
                Controller.autoThrottleButton.gameObject.SetActive(false);
            if(VSC != null)
                Controller.VSC.onValueChanged.AddListener(onVSC);
            else
                Controller.VSC.SetActive(false);
            base.init_controller();
        }

        private void onHover(bool hover)
        {
            if(hover)
                TCA.SquadConfigAction(cfg =>
                {
                    cfg.VF.XOnIfNot(VFlight.AltitudeControl);
                    cfg.BlockThrottle = true;
                });
            else
                TCA.SquadConfigAction(cfg => cfg.VF.XOffIfOn(VFlight.AltitudeControl));
        }

        void onFollowTerrain(bool follow_terrain)
        {
            CFG.AltitudeAboveTerrain = follow_terrain;
            TCA.SquadAction(tca =>
            {
                var alt = tca.GetModule<AltitudeControl>();
                alt?.SetAltitudeAboveTerrain(CFG.AltitudeAboveTerrain);
            });
        }

        private void onAutoThrottle(bool auto_throttle)
        {
            THR.BlockThrottle(auto_throttle);
        }

        private void onVSC(float vertical_speed)
        {
            VSC.SetVerticalCutoff(vertical_speed);
        }

        private void onALT(float altitude)
        {
            ALT.SetDesiredAltitude(altitude);
        }

        protected override void LocalizeHud()
        {
            HudLocalization.SetToggleLabel(Controller.hoverButton, "Hover", "Hover");
            HudLocalization.SetToggleLabel(Controller.followTerrainButton, "FollowTerrain", "Follow Terrain");
            HudLocalization.SetToggleLabel(Controller.autoThrottleButton, "AutoThrottle", "AutoThrottle");
            HudLocalization.SetToggleTooltip(Controller.hoverButton, "HoverTip", "Enable Altitude Control");
            HudLocalization.SetToggleTooltip(Controller.followTerrainButton,
                "FollowTerrainTip", "Enable follow terrain mode");
            HudLocalization.SetToggleTooltip(Controller.autoThrottleButton,
                "AutoThrottleTip", "Change altitude/vertical velocity using main throttle control");
            Controller.ALT.SetAltitudeTooltipTexts(
                Loc.T("VFlight_AltAboveGroundTip", "Desired altitude is above the ground"),
                Loc.T("VFlight_AltBelowGroundTip", "Warning! Desired altitude is below the ground"));
            foreach(var tt in Controller.GetComponentsInChildren<AT_Utils.UI.TooltipTrigger>(true))
            {
                switch(tt.text)
                {
                case "Desired vertical speed":
                    HudLocalization.SetTooltip(tt, "VFlight_VSC_Tip", tt.text);
                    break;
                case "Altitude, Vertical speed, Horizontal speed.":
                    HudLocalization.SetTooltip(tt, "VFlight_ReadoutTip", tt.text);
                    break;
                case "Change altitude or vertical speed with throttle controls":
                    HudLocalization.SetTooltip(tt, "AutoThrottleTip", tt.text);
                    break;
                case "Maintain altitude":
                    HudLocalization.SetTooltip(tt, "HoverTip", "Enable Altitude Control");
                    break;
                case "Keep altitude relative to the ground":
                    HudLocalization.SetTooltip(tt, "FollowTerrainTip", "Enable follow terrain mode");
                    break;
                }
            }
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!IsShown)
                return;
            // set controls interactable when TCA is controllable
            var controllable = TCA.IsControllable;
            Controller.hoverButton.SetInteractable(controllable);
            Controller.followTerrainButton.SetInteractable(controllable);
            Controller.autoThrottleButton.SetInteractable(controllable);
            Controller.VSC.SetInteractable(controllable);
            Controller.ALT.SetInteractable(controllable);
            // update info and controls state
            Controller.UpdateInfo(VSL.Altitude.Current,
                VSL.VerticalSpeed.Display,
                VSL.HorizontalSpeed.Absolute);
            if(ALT != null)
            {
                Controller.EnableALT(CFG.VF[VFlight.AltitudeControl]);
                Controller.ALT.SetValueWithoutNotify(CFG.DesiredAltitude);
                Controller.ALT.SetAltitudeAboveGround(VSL.Altitude.AboveGround);
                Controller.hoverButton
                    .SetIsOnAndColorWithoutNotify(CFG.VF[VFlight.AltitudeControl]);
                if(RAD != null)
                    Controller.followTerrainButton
                        .SetIsOnAndColorWithoutNotify(CFG.AltitudeAboveTerrain);
            }
            if(VSC != null)
                Controller.VSC.SetValueWithoutNotify(CFG.VerticalCutoff);
            if(THR != null)
                Controller.autoThrottleButton.SetIsOnAndColorWithoutNotify(CFG.BlockThrottle);
        }

        protected override void OnRender()
        {
            base.OnRender();
            BRC?.DrawForwardDirection();
        }
    }
}
