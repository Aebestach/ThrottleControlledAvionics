//   ManeuverUI.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri
using AT_Utils.UI;
using UnityEngine;
using UnityEngine.UI;

namespace TCA.UI
{
    public class ManeuverUI : ScreenBoundRect
    {
        public Toggle WarpToggle;
        public ManeuverInfo ManeuverInfo;
        public ManeuverSwitch ManeuverSwitch;
    }

    public class ManeuverInfo : PanelledUI
    {
        public Text Countdown, ThrustDuration;

        const float NormalPanelWidth = 90f;
        const float StatusPanelWidth = 180f;

        LayoutElement countdownLayout;
        GameObject thrustPanel;

        void Awake()
        {
            if(Countdown != null)
                countdownLayout = Countdown.transform.parent.GetComponent<LayoutElement>();
            if(ThrustDuration != null)
                thrustPanel = ThrustDuration.transform.parent.gameObject;
        }

        void setCountdownPanelWidth(float width)
        {
            if(countdownLayout != null)
                countdownLayout.preferredWidth = width;
        }

        public void UpdatePhysicsState(string message)
        {
            if(thrustPanel != null)
                thrustPanel.SetActive(false);
            setCountdownPanelWidth(StatusPanelWidth);
            Countdown.text = message;
            Countdown.color = Colors.Danger;
        }

        public void UpdateInfo(float countdown, float ttb)
        {
            if(thrustPanel != null)
                thrustPanel.SetActive(true);
            setCountdownPanelWidth(NormalPanelWidth);
            Countdown.text = countdown >= 0 ? string.Format("{0:F1}s", countdown) : "";
            Countdown.color = countdown > 10 ? Colors.Neutral : Colors.Danger;
            if(ttb >= 0 && ttb < float.MaxValue)
            {
                ThrustDuration.gameObject.SetActive(true);
                ThrustDuration.text = string.Format("{0:F1}s", ttb);
            }
            else
                ThrustDuration.gameObject.SetActive(false);
        }
    }

    public class ManeuverSwitch : PanelledUI
    {
        public Button Button;
        public Text ButtonText;
        public string AbortLabel = "Abort Maneuver";
        public string ExecuteLabel = "Execute Node";
        bool maneuverActive;

        void Awake()
        {
            SetManeuverActive(false, true);
        }

        void onButtonColorChange(Color color)
        {
            ButtonText.color = color;
        }

        public void SetManeuverActive(bool active, bool force = false)
        {
            if(force || maneuverActive != active)
            {
                maneuverActive = active;
                if(active)
                {
                    ButtonText.text = AbortLabel;
                    Colors.Active.removeOnColorChangeListner(onButtonColorChange);
                    Colors.Danger.addOnColorChangeListner(onButtonColorChange);
                    onButtonColorChange(Colors.Danger);
                }
                else
                {
                    ButtonText.text = ExecuteLabel;
                    Colors.Danger.removeOnColorChangeListner(onButtonColorChange);
                    Colors.Active.addOnColorChangeListner(onButtonColorChange);
                    onButtonColorChange(Colors.Active);
                }
            }
        }
    }
}
