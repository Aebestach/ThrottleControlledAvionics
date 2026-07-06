using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class InfoPanel : ControlPanel<InfoUI>
    {
        public static string TEST_MSG => Loc.T("InfoPanelTestMsg", "Drag this to set the location of TCA status panel");

        private void clearGUIStatus()
        {
            TCAGui.ClearStatus();
            unlockControls();
        }

        protected override void LocalizeHud()
        {
            foreach(var tt in Controller.GetComponentsInChildren<AT_Utils.UI.TooltipTrigger>(true))
            {
                if(tt.text == "Click to dismiss")
                    HudLocalization.SetTooltip(tt, "Info_DismissTip", tt.text);
            }
        }

        protected override void init_controller()
        {
            Controller.message.onLabelClicked.AddListener(clearGUIStatus);
            if(!initialized)
                Controller.SetMessage(TEST_MSG);
            base.init_controller();
        }

        public override void Reset()
        {
            if(Controller != null)
                Controller.message.onLabelClicked.RemoveListener(clearGUIStatus);
            base.Reset();
        }

        public void ClearMessage()
        {
            if(Controller == null)
                return;
            Controller.ClearMessage();
            unlockControls();
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!IsShown)
                return;
            if(!string.IsNullOrEmpty(TCAGui.StatusMessage))
                Controller.SetMessage(TCAGui.StatusMessage);
        }
    }
}
