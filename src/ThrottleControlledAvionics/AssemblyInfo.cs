using System;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class ModInfo : KSP_AVC_Info
    {
        public ModInfo()
        {
            MinKSPVersion = new Version(1, 12, 3);
            MaxKSPVersion = new Version(1, 12, 3);

            VersionURL = "https://raw.githubusercontent.com/Aebestach/ThrottleControlledAvionics/master/GameData/ThrottleControlledAvionics/ThrottleControlledAvionics.version";
            UpgradeURL = "http://spacedock.info/mod/198/Throttle%20Controlled%20Avionics";
            ChangeLogURL = "https://raw.githubusercontent.com/Aebestach/ThrottleControlledAvionics/master/ChangeLog.md";
        }
    }
}
