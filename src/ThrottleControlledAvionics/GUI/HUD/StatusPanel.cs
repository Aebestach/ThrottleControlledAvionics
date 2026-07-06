using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class StatusPanel : ControlPanel<StatusUI>
    {
        private VerticalSpeedControl VSC;
        private AltitudeControl ALT;
        private VTOLAssist VTOL_assist;
        private VTOLControl VTOL_control;
        private FlightStabilizer Stabilizer;
        private CollisionPreventionSystem CPS;
        private Radar RAD;
        private HorizontalSpeedControl HSC;
        private PointNavigator NAV;
        private Anchor anchor;

        protected override void init_controller()
        {
            if(VSC == null)
                Controller.VSC.Show(false);
            if(ALT == null)
                Controller.ALT.Show(false);
            if(VTOL_assist == null)
                Controller.VTOLAssist.Show(false);
            if(VTOL_control == null)
                Controller.VTOLMode.Show(false);
            if(Stabilizer == null)
                Controller.Stabilizing.Show(false);
            if(CPS == null)
                Controller.VesselCollision.Show(false);
            if(RAD == null)
                Controller.TerrainCollision.Show(false);
            if(NAV == null)
                Controller.Navigation.Show(false);
            if(HSC == null)
                Controller.Stop.Show(false);
            base.init_controller();
        }

        protected override void onGamePause()
        {
            base.onGamePause();
            if(Controller != null)
                Controller.PauseSound(true);
        }

        protected override void onGameUnpause()
        {
            base.onGameUnpause();
            if(Controller != null)
                Controller.PauseSound(false);
        }

        protected override void LocalizeHud()
        {
            HudLocalization.SetIndicatorTooltip(Controller.VSC,
                "Status_VSC_Tip", "Vertical speed control");
            HudLocalization.SetIndicatorTooltip(Controller.NoEC,
                "Status_NoEC_Tip", "No electric charge");
            HudLocalization.SetIndicatorTooltip(Controller.VesselCollision,
                "Status_VesselCollision_Tip", "Collision with another vessel");
            HudLocalization.SetIndicatorTooltip(Controller.TerrainCollision,
                "Status_TerrainCollision_Tip", "Collision with terrain");
            HudLocalization.SetIndicatorTooltip(Controller.Navigation,
                "Status_Navigation_Tip", "Automatic navigation");
            HudLocalization.SetIndicatorTooltip(Controller.Ascending,
                "Status_Ascending_Tip", "Gaining altitude");
            HudLocalization.SetIndicatorTooltip(Controller.NoEngines,
                "Status_NoEngines_Tip", "No active engines");
            HudLocalization.SetIndicatorTooltip(Controller.SmartEngines,
                "SmartEnginesTip", "Group engines by thrust direction and automatically use appropriate group for a maneuver");
            HudLocalization.SetIndicatorTooltip(Controller.LowControlAuthority,
                "Status_LowControl_Tip", "Low control authority");
            HudLocalization.SetIndicatorTooltip(Controller.ALT,
                "Status_ALT_Tip", "Altitude control");
            HudLocalization.SetIndicatorTooltip(Controller.EnginesUnoptimized,
                "Status_EnginesUnoptimized_Tip", "Unbalanced engines");
            HudLocalization.SetIndicatorTooltip(Controller.LoosingAltitude,
                "Status_LosingAltitude_Tip", "Losing altitude");
            HudLocalization.SetIndicatorTooltip(Controller.VTOLMode,
                "Status_VTOLMode_Tip", "Copter-style control mode");
            HudLocalization.SetIndicatorTooltip(Controller.Stop,
                "Status_Stop_Tip", "Stop or Anchor");
            HudLocalization.SetIndicatorTooltip(Controller.Stabilizing,
                "Status_Stabilizing_Tip", "Stabilizing VTOL flight");
            HudLocalization.SetIndicatorTooltip(Controller.VTOLAssist,
                "VTOLAssistTipAdv", "Assist with vertical takeoff and landing");
            HudLocalization.SetToggleTooltip(Controller.soundToggle,
                "Status_SoundTip", "Toggle alert sounds");
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!IsShown)
                return;
            // disable sub-panels depending on situation
            Controller.ToggleOnPlanet(VSL.OnPlanet);
            Controller.ToggleInOrbit(VSL.InOrbit);
            // set states of the indicators
            Controller.Ascending.isOn = TCA.IsStateSet(TCAState.Ascending);
            Controller.LoosingAltitude.isOn = TCA.IsStateSet(TCAState.LoosingAltitude);
            Controller.TerrainCollision.isOn = TCA.IsStateSet(TCAState.GroundCollision);
            Controller.VesselCollision.isOn = TCA.IsStateSet(TCAState.VesselCollision);
            Controller.LowControlAuthority.isOn = !VSL.Controls.HaveControlAuthority;
            Controller.EnginesUnoptimized.isOn = TCA.IsStateSet(TCAState.Unoptimized);
            Controller.VSC.isOn = TCAModule.ExistsAndActive(VSC);
            Controller.ALT.isOn = TCAModule.ExistsAndActive(ALT);
            Controller.VTOLMode.isOn = TCAModule.ExistsAndActive(VTOL_control);
            Controller.VTOLAssist.isOn = TCA.IsStateSet(TCAState.VTOLAssist);
            Controller.Stabilizing.isOn = TCA.IsStateSet(TCAState.StabilizeFlight);
            Controller.NoEngines.isOn = TCA.IsStateSet(TCAState.HaveEC)
                                        && !TCA.IsStateSet(TCAState.HaveActiveEngines);
            Controller.NoEC.isOn = TCA.IsStateSet(TCAState.Enabled)
                                   && !TCA.IsStateSet(TCAState.HaveEC);
            Controller.SmartEngines.isOn = CFG.UseSmartEngines;
            Controller.Stop.isOn = CFG.HF[HFlight.Stop] 
                                   || TCAModule.ExistsAndActive(anchor);
            Controller.Navigation.isOn = CFG.Nav.Any(Navigation.GoToTarget,
                Navigation.FollowPath,
                Navigation.FollowTarget);
            // fade out irrelevant indicators
            Controller.TerrainCollision.SetActive(TCAModule.ExistsAndActive(RAD));
            Controller.VesselCollision.SetActive(CFG.UseCPS);
            Controller.VTOLAssist.SetActive(CFG.VTOLAssistON);
            Controller.Stabilizing.SetActive(CFG.StabilizeFlight);
            Controller.SmartEngines.SetActive(VSL.Engines.Clusters.Multi);
        }
    }
}
