//   TCAGlobals.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri

using System;
using System.IO;
using System.Reflection;
using System.Linq;
using System.Collections.Generic;
using KSP.Localization;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    class Globals : PluginGlobals<Globals>
    {
        public const string TCA_PART = "ThrottleControlledAvionics";
        public const string INSTRUCTIONS = "INSTRUCTIONS.md";
        public const string INSTRUCTIONS_ZH = "INSTRUCTIONS.zh-cn.md";

        public const string RADIATION_ICON = "ThrottleControlledAvionics/Icons/waypoint";
        public const string CIRCLE_ICON = "ThrottleControlledAvionics/Icons/path-node";

        public UIBundle AssetBundle = UIBundle.Create("ThrottleControlledAvionics/tca_ui.bundle");

        [Persistent] public bool  IntegrateIntoCareer   = true;
        [Persistent] public bool  RoleSymmetryInFlight  = true;
        [Persistent] public bool  UseStockAppLauncher   = false;
        [Persistent] public bool  AutosaveBeforeLanding = true;

        [Persistent] public float InputDeadZone        = 0.01f; //1% of steering or translation control
        [Persistent] public int   MaxManualGroups      = 10; //maximum number of manual control groups
        [Persistent] public float KeyRepeatTime        = 0.1f;
        [Persistent] public float ClickDuration        = 0.05f;
        [Persistent] public float WaypointFadoutDist   = 10000f;
        [Persistent] public float CameraFadeinPower    = 0.3f;
        [Persistent] public float UnpackDistance       = 5000f;
        [Persistent] public float ActionListHeight     = 110f;
        [Persistent] public float MaxAAFilter          = 1f;
        [Persistent] public float ExhaustSafeDist      = 1.1f;
        [Persistent] public float HudUiScale           = 0.65f;

        [Persistent] public string PersistentRotationName = "PersistentRotation";
        [Persistent] public float PersistentRotationThreshold = 5e-7f;
        [Persistent] public float NoPersistentRotationThreshold = 5e-7f;

        public MDSection Manual;
        public static readonly SortedDictionary<string, Type> AllConfigs;

        static Globals()
        {
            AllConfigs = new SortedDictionary<string, Type>();
            foreach(var cmp in Assembly.GetExecutingAssembly().GetTypes()
                    .Where(t => t.IsSubclassOf(typeof(TCAComponent))))
            {
                foreach(var cfg in cmp.GetNestedTypes()
                        .Where(t => !t.IsAbstract && t.IsSubclassOf(typeof(TCAComponent.ComponentConfig))))
                    AllConfigs.Add(cmp.Name, cfg);
            }
        }

        static FieldInfo get_INST(Type t) => 
        t.GetField("INST", BindingFlags.Static|BindingFlags.FlattenHierarchy|BindingFlags.Public);

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            foreach(var config in AllConfigs)
            {
                var config_node = node.GetNode(config.Key);
                if(config_node != null)
                {
                    var INSTf = get_INST(config.Value);
                    if(INSTf != null)
                    {
                        if(INSTf.GetValue(null) is TCAComponent.ComponentConfig INST)
                            INST.Load(config_node);
                    }
                    else
                        Utils.Log("WARNING: {} has no public static INST field", config.Value.FullName);
                }
#if DEBUG
                else
                    Utils.Log("WARNING: no configuration for {}", config.Key);
#endif
            }
        }

        public override void Save(ConfigNode node)
        {
            base.Save(node);
            foreach(var config in AllConfigs)
            {
                var INSTf = get_INST(config.Value);
                if(INSTf != null)
                {
                    var INST = INSTf.GetValue(null) as TCAComponent.ComponentConfig;
                    if(INST != null)
                        INST.Save(node.AddNode(config.Key));
                }
            }
        }

        public override void Init()
        { 
            LoadManual();
            InputDeadZone *= InputDeadZone; //it is compared with the sqrMagnitude
        }

        static string InstructionsFile()
        {
            var lang = Localizer.CurrentLanguage?.ToLowerInvariant() ?? "en-us";
            if(lang.StartsWith("zh"))
                return INSTRUCTIONS_ZH;
            return INSTRUCTIONS;
        }

        public void LoadManual()
        {
            Manual = null;
            var candidates = new[] { InstructionsFile(), INSTRUCTIONS };
            foreach(var file in candidates.Distinct())
            {
                var path = PluginFolder(file);
                if(!File.Exists(path)) continue;
                try
                {
                    using(var reader = new StreamReader(path))
                    {
                        Manual = MD2Unity.Parse(reader);
                        if(Manual.NoTitle)
                            Manual.Title = Loc.T("ManualWindowTitle", "TCA Manual");
                    }
                    return;
                }
                catch(Exception ex) { Utils.Log("Error loading {} file:\n{}", path, ex); }
            }
        }
    }
}

