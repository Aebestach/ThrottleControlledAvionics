//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.IO;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart(typeof(PointNavigator))]
    [RequireModules(typeof(HorizontalSpeedControl),
                    typeof(VerticalSpeedControl),
                    typeof(AttitudeControl))]
    [OptionalModules(typeof(TranslationControl),
                     typeof(CollisionPreventionSystem),
                     typeof(AltitudeControl))]
    public class HoverDockingAutopilot : TCAModule
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float TakeoffAltitude = 50f;
            [Persistent] public float TakeoffVerticalSpeed = 3f;
            [Persistent] public float CoarseStandoff = 50f;
            [Persistent] public float HoldStandoff = 8f;
            [Persistent] public float ContactOffset = 0.75f;
            [Persistent] public float HoldTolerance = 1.5f;
            [Persistent] public float MaxCoarseSpeed = 25f;
            [Persistent] public float MaxHoldSpeed = 2f;
            [Persistent] public float MaxFinalSpeed = 0.4f;
            [Persistent] public float FinalApproachSpeed = 0.2f;
            [Persistent] public float MaxTargetSpeed = 2f;
            [Persistent] public float MaxLateralError = 0.35f;
            [Persistent] public float MaxVerticalError = 0.35f;
            [Persistent] public float MaxAlignmentAngle = 5f;
            [Persistent] public float MaxRollCaptureAngle = 15f;
            [Persistent] public float FinalCaptureAlignmentAngle = 15f;
            [Persistent] public float MaxSurfaceAlignmentTilt = 35f;
            [Persistent] public float AbortDistance = 250f;
            [Persistent] public float PositionGain = 0.35f;
            [Persistent] public float VerticalGain = 0.35f;
            [Persistent] public float VelocityCorrection = 0.75f;
            [Persistent] public float FinalLateralFunnel = 2f;
            [Persistent] public float FinalMinSpeed = 0.03f;
            [Persistent] public float FinalAlignmentHoldTime = 1.5f;
            [Persistent] public float TerminalCorridorHysteresis = 1.5f;
            [Persistent] public float TerminalSevereDriftFactor = 6f;
            [Persistent] public float TerminalRcsMinTWR = 1.15f;
            [Persistent] public float TerminalRcsLowGravity = 0.05f;
            [Persistent] public float TerminalRcsMinControlAccel = 0.05f;
            [Persistent] public float TerminalTiltGainP = 0.4f;
            [Persistent] public float TerminalTiltGainD = 0.9f;
            [Persistent] public float MaxTerminalTilt = 3f;
            [Persistent] public float FreeFallDistanceAtmosphere = 8f;
            [Persistent] public float FreeFallDistanceVacuum = 5f;
        }
        public static Config C => Config.INST;

        public enum Stage
        {
            None,
            ResolveTarget,
            Takeoff,
            CoarseApproach,
            Standoff,
            Align,
            HoldAndAlign,
            TerminalHold,
            TerminalApproach,
            FinalApproach,
            Docked,
            Aborted,
            Finished
        }

        public enum TerminalControlMode
        {
            Auto,
            HoverAssist,
            RcsOnly,
            Disabled
        }

        [Persistent] public Stage stage;
        [Persistent] public bool ShowOptions;
        [Persistent] public bool OptionsInitialized;
        [Persistent] public bool AlignRoll = true;
        [Persistent] public bool AutoEnableRcs = true;
        [Persistent] public TerminalControlMode TerminalMode = TerminalControlMode.Auto;
        [Persistent] public FloatField HoverAltitude = new FloatField("F0", 10, 1000);
        [Persistent] public FloatField RollOffset = new FloatField("F0", -180, 180, circle: true);
        [Persistent] public FloatField HoldStandoff = new FloatField("F1", 2, 50);
        [Persistent] public FloatField MaxCoarseSpeed = new FloatField("F1", 5, 100);
        [Persistent] public FloatField FinalApproachSpeed = new FloatField("F2", 0.05f, 2);
        [Persistent] public FloatField MaxTargetSpeed = new FloatField("F1", 0.5f, 10);
        [Persistent] public FloatField MaxAlignmentAngle = new FloatField("F1", 1, 30);
        // Keep the upper bound tight: a large contact offset silently moves the docking
        // point meters away from the target port and the ports never reach capture range.
        [Persistent] public FloatField ContactOffset = new FloatField("F2", 0.1f, 2);
        [Persistent] public FloatField FreeFallDistanceAtmosphere = new FloatField("F1", 0, 15);
        [Persistent] public FloatField FreeFallDistanceVacuum = new FloatField("F1", 0, 15);

        #pragma warning disable 169
        HorizontalSpeedControl HSC;
        VerticalSpeedControl VSC;
        AttitudeControl ATC;
        AltitudeControl ALT;
        TranslationControl TRA;
        CollisionPreventionSystem CPS;
        #pragma warning restore 169

        WayPoint target;
        ITargetable targetObject;
        Vessel targetVessel;
        Transform targetTransform;
        bool dockingTarget;
        bool finalCpsExemption;
        bool rcsEnabledByAutopilot;
        Vector3d approachAxis;
        DockingTargetResolver dockingResolver;
        DockingPoseController poseController;
        PredictiveBrakingController brakingController;
        DockingPoseMetrics poseMetrics;
        readonly AsymmetricFiterF coarse_max_speed = new AsymmetricFiterF();
        readonly PIDf_Controller CoarseDistancePID = new PIDf_Controller();
        readonly PIDf_Controller CoarseCorrectionPID = new PIDf_Controller();
        string statusKey = "HoverDocking_StatusReady";
        string statusFallback = "Ready.";
        Stage lastDebugStage = Stage.None;
        string lastDebugAttitudeMode = "";
        float nextDebugSnapshotTime;
        float nextDebugAttitudeTime;
        float nextDebugTranslationTime;
        float nextDebugPortsTime;
        float nextDebugFinalHoldTime;
        float nextDebugTerminalModeTime;
        float finalStableSince = -1;
        TerminalControlMode resolvedTerminalMode = TerminalControlMode.Auto;
        bool rcsOnlyTerminalActive;
        bool freeFallTerminalActive;
        bool debug_log_guard;
        bool cleanupInProgress;
        StreamWriter debug_log_writer;
        string debug_log_path;

        // Write to HoverDocking.debug.log in the KSP folder ??never UnityEngine.Debug.Log here;
        // ModuleManager's log interceptor deadlocks when many logs fire during FixedUpdate.
        const bool HoverDockingDebug = true;

        public float Distance { get; private set; }
        public float LateralError { get; private set; }
        public float VerticalError { get; private set; }
        public float AxialDistance { get; private set; }
        public float LateralSpeed { get; private set; }
        public float ClosingSpeed { get; private set; }
        public float AlignmentAngle { get; private set; }
        public float RollError { get; private set; }
        public float FinalCorridor { get; private set; }
        public bool TargetCpsExempt => finalCpsExemption;

        public HoverDockingAutopilot(ModuleTCA tca) : base(tca) {}

        public override void Init()
        {
            base.Init();
            CFG.Nav.AddCallback(HoverDockingCallback, Navigation.HoverDocking);
            coarse_max_speed.TauUp = PointNavigator.C.MaxSpeedFilterUp;
            coarse_max_speed.TauDown = PointNavigator.C.MaxSpeedFilterDown;
            CoarseDistancePID.setPID(PointNavigator.C.DistancePID);
            CoarseCorrectionPID.setPID(PointNavigator.C.CorrectionPID);
            dockingResolver = new DockingTargetResolver(VSL);
            poseController = new DockingPoseController(VSL, TRA, ATC);
            brakingController = new PredictiveBrakingController(VSL, HSC);
            init_option_defaults();
        }

        void init_option_defaults()
        {
            if(HoverAltitude.Value < 10)
                HoverAltitude.Value = C.TakeoffAltitude;
            if(OptionsInitialized)
                return;
            HoverAltitude.Value = C.TakeoffAltitude;
            HoldStandoff.Value = C.HoldStandoff;
            MaxCoarseSpeed.Value = C.MaxCoarseSpeed;
            FinalApproachSpeed.Value = C.FinalApproachSpeed;
            MaxTargetSpeed.Value = C.MaxTargetSpeed;
            MaxAlignmentAngle.Value = C.MaxAlignmentAngle;
            ContactOffset.Value = C.ContactOffset;
            FreeFallDistanceAtmosphere.Value = C.FreeFallDistanceAtmosphere;
            FreeFallDistanceVacuum.Value = C.FreeFallDistanceVacuum;
            OptionsInitialized = true;
        }

        public override void Disable()
        {
            finish(false);
        }

        WayPoint prefer_docking_port_target(WayPoint wp)
        {
            if(wp == null)
                return null;
            var kspTarget = VSL.vessel.targetObject;
            if(kspTarget is ModuleDockingNode port)
            {
                var wpTarget = wp.GetTarget();
                if(!(wpTarget is ModuleDockingNode))
                {
                    var portVessel = port.GetVessel();
                    var wpVessel = wpTarget != null ? wpTarget.GetVessel() : null;
                    if(portVessel != null && (wpVessel == null || wpVessel == portVessel))
                        return new WayPoint(port);
                }
            }
            return wp;
        }

        protected override void UpdateState()
        {
            base.UpdateState();
            IsActive &= VSL.OnPlanet && CFG.Nav[Navigation.HoverDocking] && CFG.Target;
        }

        public void HoverDockingCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
            case Multiplexer.Command.On:
                var wp = VSL.ResolveTarget() ?? VSL.TargetAsWP;
                if(wp == null) abort("HoverDocking_NoTarget", "No docking target selected.");
                else start(wp);
                break;
            case Multiplexer.Command.Off:
                finish(false);
                break;
            }
        }

        public bool HasUsableTarget()
        {
            var wp = VSL.ResolveTarget() ?? VSL.TargetAsWP;
            if(wp == null)
                return false;
            wp.Update(VSL);
            if(!wp)
                return false;
            var target = wp.GetTarget();
            var vessel = target != null ? target.GetVessel() : null;
            return vessel != null && vessel != VSL.vessel && vessel.mainBody == VSL.Body;
        }

        void start(WayPoint wp)
        {
            cleanupInProgress = false;
            wp = prefer_docking_port_target(wp);
            target = wp;
            target.Update(VSL);
            SetTarget(target);
            reset_target();
            NeedCPS();
            CoarseDistancePID.Reset();
            CoarseCorrectionPID.Reset();
            coarse_max_speed.Set(MaxCoarseSpeed.Value);
            brakingController?.Reset(MaxCoarseSpeed.Value);
            poseController?.Reset();
            CFG.BlockThrottle = true;
            CFG.VerticalCutoff = 0;
            CFG.VF.OffIfOn(VFlight.AltitudeControl);
            CFG.HF.OnIfNot(HFlight.Level);
            statusKey = "HoverDocking_StatusResolving";
            statusFallback = "Resolving docking target.";
            stage = Stage.ResolveTarget;
            resolvedTerminalMode = TerminalMode;
            rcsOnlyTerminalActive = false;
            freeFallTerminalActive = false;
            reset_debug_log_state();
            open_debug_log();
            debug_log("START target={} targetVessel={} autoRCS={} terminalMode={} alignRoll={} hover={} hold={} finalSpeed={} maxAlign={} contactOffset={} freeFallAtmo={} freeFallVac={} rollOffset={}",
                targetObject != null ? targetObject.GetName() : "unresolved",
                targetVessel != null ? targetVessel.vesselName : "unresolved",
                AutoEnableRcs,
                TerminalMode,
                AlignRoll,
                HoverAltitude.Value,
                HoldStandoff.Value,
                FinalApproachSpeed.Value,
                MaxAlignmentAngle.Value,
                ContactOffset.Value,
                FreeFallDistanceAtmosphere.Value,
                FreeFallDistanceVacuum.Value,
                RollOffset.Value);
        }

        void finish(bool docked)
        {
            if(cleanupInProgress)
                return;
            cleanupInProgress = true;
            debug_log("FINISH docked={} stage={} dist={} lateral={} vertical={} axial={} align={} roll={} rcsGroup={} rcsByAP={} cpsExempt={}",
                docked,
                stage,
                Distance,
                LateralError,
                VerticalError,
                AxialDistance,
                AlignmentAngle,
                RollError,
                VSL.vessel.ActionGroups[KSPActionGroup.RCS],
                rcsEnabledByAutopilot,
                finalCpsExemption);
            restore_hover_support();
            restore_rcs();
            set_cps_exemption(false);
            close_debug_log();
            ReleaseCPS();
            StopUsingTarget();
            target = null;
            targetObject = null;
            targetVessel = null;
            targetTransform = null;
            dockingTarget = false;
            finalCpsExemption = false;
            rcsOnlyTerminalActive = false;
            freeFallTerminalActive = false;
            approachAxis = Vector3d.zero;
            dockingResolver?.Reset();
            poseController?.Reset();
            if(CFG.Nav[Navigation.HoverDocking])
                CFG.Nav.OffIfOn(Navigation.HoverDocking);
            CFG.VerticalCutoff = 0;
            if(docked)
            {
                stage = Stage.Docked;
                statusKey = "HoverDocking_StatusDocked";
                statusFallback = "Docked.";
            }
            else if(stage != Stage.Aborted)
            {
                stage = Stage.Finished;
                statusKey = "HoverDocking_StatusFinished";
                statusFallback = "Hover docking finished.";
            }
            CFG.HF.OnIfNot(HFlight.Stop);
            cleanupInProgress = false;
        }

        void abort(string key, string fallback)
        {
            if(cleanupInProgress)
                return;
            cleanupInProgress = true;
            debug_log("ABORT key={} fallback={} stage={} dist={} lateral={} vertical={} axial={} align={} roll={} rcsGroup={} rcsByAP={} cpsExempt={}",
                key,
                fallback,
                stage,
                Distance,
                LateralError,
                VerticalError,
                AxialDistance,
                AlignmentAngle,
                RollError,
                VSL.vessel.ActionGroups[KSPActionGroup.RCS],
                rcsEnabledByAutopilot,
                finalCpsExemption);
            restore_hover_support();
            restore_rcs();
            set_cps_exemption(false);
            close_debug_log();
            ReleaseCPS();
            StopUsingTarget();
            statusKey = key;
            statusFallback = fallback;
            stage = Stage.Aborted;
            if(CFG.Nav[Navigation.HoverDocking])
                CFG.Nav.OffIfOn(Navigation.HoverDocking);
            CFG.VerticalCutoff = 0;
            CFG.HF.OnIfNot(HFlight.Stop);
            Message(Loc.T(key, fallback));
            cleanupInProgress = false;
        }

        void reset_target()
        {
            targetObject = null;
            targetVessel = null;
            targetTransform = null;
            dockingTarget = false;
            approachAxis = Vector3d.zero;
            dockingResolver?.Reset();
        }

        void reset_debug_log_state()
        {
            lastDebugStage = Stage.None;
            lastDebugAttitudeMode = "";
            nextDebugSnapshotTime = 0;
            nextDebugAttitudeTime = 0;
            nextDebugTranslationTime = 0;
            nextDebugPortsTime = 0;
            nextDebugFinalHoldTime = 0;
            nextDebugTerminalModeTime = 0;
            finalStableSince = -1;
            rcsOnlyTerminalActive = false;
            freeFallTerminalActive = false;
        }

        void open_debug_log()
        {
            close_debug_log();
            debug_log_path = Path.Combine(KSPUtil.ApplicationRootPath, "HoverDocking.debug.log");
            debug_log_writer = new StreamWriter(debug_log_path, true);
            debug_log_writer.AutoFlush = true;
            debug_log_writer.WriteLine();
            debug_log_writer.WriteLine("=== HoverDocking session " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss")
                                       + " vessel=" + (VSL.vessel != null ? VSL.vessel.vesselName : "?")
                                       + " log=" + debug_log_path + " ===");
        }

        void close_debug_log()
        {
            if(debug_log_writer == null)
                return;
            try
            {
                debug_log_writer.WriteLine("=== session end " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + " ===");
                debug_log_writer.Dispose();
            }
            catch { /* ignore close errors */ }
            debug_log_writer = null;
        }

        static string port_desc(ModuleDockingNode port)
        {
            if(port == null || port.part == null)
                return "none";
            var title = port.part.partInfo != null ? port.part.partInfo.title : port.part.name;
            return title + " [" + port.part.name + " id=" + port.part.flightID + "]";
        }

        static float vec_angle(Vector3 a, Vector3 b)
        {
            return a.sqrMagnitude > 1e-6f && b.sqrMagnitude > 1e-6f ? Utils.Angle2(a, b) : -1;
        }

        void debug_docking_ports(string reason, Vector3d axis, bool force = false)
        {
            if(!force && Time.time < nextDebugPortsTime)
                return;
            nextDebugPortsTime = Time.time + 3f;
            var frame = docking_frame;
            var targetPort = frame?.TargetPort;
            var activePort = frame?.ActivePort;
            var refPart = VSL.vessel.GetReferenceTransformPart();
            var refPort = refPart != null ? refPart.FindModuleImplementing<ModuleDockingNode>() : null;
            var origTarget = frame?.OriginalTarget;
            var kspPort = FlightGlobals.fetch != null ? FlightGlobals.fetch.VesselTarget as ModuleDockingNode : null;
            var explicitTargetPort = origTarget is ModuleDockingNode || kspPort != null;
            var targetTr = frame?.TargetTransform;
            var activeTr = frame?.ActiveTransform;
            var targetForward = targetTr != null ? targetTr.forward : Vector3.zero;
            var targetUp = targetTr != null ? targetTr.up : Vector3.zero;
            var activeForward = activeTr != null ? activeTr.forward : Vector3.zero;
            var activeUp = activeTr != null ? activeTr.up : Vector3.zero;
            var neededForward = axis.sqrMagnitude > 1e-6 ? (Vector3)(-axis).normalized : Vector3.zero;
            var targetGetFwd = frame?.TargetObject != null ? frame.TargetObject.GetFwdVector() : Vector3.zero;
            var negTargetForward = targetForward.sqrMagnitude > 1e-6f ? -targetForward : Vector3.zero;
            debug_log("PORTS {} explicitTargetPort={} origTarget={} kspPort={} targetVessel={} activeVessel={} refPart={} refIsPort={} activeFromRef={}",
                reason,
                explicitTargetPort,
                origTarget != null ? origTarget.GetName() : "none",
                kspPort != null ? port_desc(kspPort) : "none",
                frame?.TargetVessel != null ? frame.TargetVessel.vesselName : "none",
                VSL.vessel.vesselName,
                refPart != null ? refPart.name : "none",
                refPort != null,
                refPort != null && activePort != null && refPort == activePort);
            debug_log("PORTS TARGET {} pos={} fwd={} up={} getFwd={} axis={} fwdVsAxis={}?",
                port_desc(targetPort),
                vec(frame != null ? frame.TargetPosition : Vector3d.zero),
                vec(targetForward),
                vec(targetUp),
                vec(targetGetFwd),
                vec(axis),
                vec_angle(targetForward, (Vector3)axis.normalized));
            debug_log("PORTS ACTIVE {} pos={} fwd={} up={} neededFwd={} alignDelta={}? activeVsNegTarget={}? (MJ:0?=docked)",
                port_desc(activePort),
                vec(frame != null ? frame.ActivePosition : Vector3d.zero),
                vec(activeForward),
                vec(activeUp),
                vec(neededForward),
                alignment_delta(axis),
                vec_angle(activeForward, negTargetForward));
        }

        void debug_log(string msg, params object[] args)
        {
            if(!HoverDockingDebug || debug_log_guard)
                return;
            debug_log_guard = true;
            try
            {
                if(debug_log_writer == null)
                    open_debug_log();
                if(args.Length > 0)
                    msg = Utils.Format(msg, args);
                debug_log_writer.WriteLine("[" + DateTime.Now.ToString("HH:mm:ss.fff") + "] " + msg);
            }
            catch { /* never break autopilot because logging failed */ }
            finally
            {
                debug_log_guard = false;
            }
        }

        static string vec(Vector3d v)
        {
            return string.Format("({0:F2},{1:F2},{2:F2})", v.x, v.y, v.z);
        }

        static string vec(Vector3 v)
        {
            return string.Format("({0:F2},{1:F2},{2:F2})", v.x, v.y, v.z);
        }

        void debug_snapshot(string reason, Vector3d axis, bool force = false)
        {
            if(!force && Time.time < nextDebugSnapshotTime)
                return;
            nextDebugSnapshotTime = Time.time + 1.5f;
            var activeTransform = docking_frame != null ? docking_frame.ActiveTransform : null;
            var activeForward = activeTransform != null
                ? activeTransform.forward
                : VSL.refT != null ? VSL.refT.forward : Vector3.zero;
            var activeUp = activeTransform != null
                ? activeTransform.up
                : VSL.refT != null ? VSL.refT.up : Vector3.zero;
            var alignDelta = alignment_delta(axis);
            var neededForward = axis.sqrMagnitude > 1e-6 ? (Vector3)(-axis).normalized : Vector3.zero;
            var forwardError = neededForward.sqrMagnitude > 1e-6f && activeForward.sqrMagnitude > 1e-6f
                ? Utils.Angle2(activeForward, neededForward)
                : -1;
            debug_log("{}: stage={} status={} dist={} lat={} vert={} axial={} latV={} closeV={} align={} roll={} corridor={} terminal={} resolvedTerminal={} rcsOnly={} axis={} alignDelta={} safe={} rcsGroup={} rcsByAP={} autoRCS={} TRA={} transAvail={} cpsUse={} cpsExempt={} target={} targetLanded={} targetSpeed={} activeF={} activeUp={} neededF={} fwdErr={}",
                reason,
                stage,
                statusKey,
                Distance,
                LateralError,
                VerticalError,
                AxialDistance,
                LateralSpeed,
                ClosingSpeed,
                AlignmentAngle,
                RollError,
                FinalCorridor,
                TerminalMode,
                resolvedTerminalMode,
                rcsOnlyTerminalActive,
                vec(axis),
                alignDelta,
                axis.sqrMagnitude > 1e-6 && alignment_safe(axis),
                VSL.vessel.ActionGroups[KSPActionGroup.RCS],
                rcsEnabledByAutopilot,
                AutoEnableRcs,
                TRA != null,
                VSL.Controls.TranslationAvailable,
                CFG.UseCPS,
                finalCpsExemption,
                targetVessel != null ? targetVessel.vesselName : "none",
                targetVessel != null && targetVessel.LandedOrSplashed,
                targetVessel != null ? targetVessel.srf_velocity.magnitude : 0,
                vec(activeForward),
                vec(activeUp),
                vec(neededForward),
                forwardError);
            debug_docking_ports(reason, axis, force);
        }

        void debug_stage_transition(Stage from, Stage to, Vector3d axis)
        {
            if(from == to)
                return;
            debug_log("STAGE {} -> {} status={} dist={} lat={} vert={} axial={} align={} roll={}",
                from,
                to,
                statusKey,
                Distance,
                LateralError,
                VerticalError,
                AxialDistance,
                AlignmentAngle,
                RollError);
            lastDebugStage = to;
            debug_docking_ports("stage-" + to, axis, true);
        }

        void debug_attitude_command(string mode, Vector3d axis)
        {
            if(mode == lastDebugAttitudeMode && Time.time < nextDebugAttitudeTime)
                return;
            lastDebugAttitudeMode = mode;
            nextDebugAttitudeTime = Time.time + 0.5f;
            var activeTransform = docking_frame != null ? docking_frame.ActiveTransform : null;
            var activeForward = activeTransform != null
                ? activeTransform.forward
                : VSL.refT != null ? VSL.refT.forward : Vector3.zero;
            var activeUp = activeTransform != null
                ? activeTransform.up
                : VSL.refT != null ? VSL.refT.up : Vector3.zero;
            var targetForward = targetTransform != null ? targetTransform.forward : Vector3.zero;
            var targetUp = targetTransform != null ? targetTransform.up : Vector3.zero;
            var alignDelta = alignment_delta(axis);
            var neededForward = axis.sqrMagnitude > 1e-6 ? (Vector3)(-axis).normalized : Vector3.zero;
            var forwardError = neededForward.sqrMagnitude > 1e-6f && activeForward.sqrMagnitude > 1e-6f
                ? Utils.Angle2(activeForward, neededForward)
                : -1;
            debug_log("ATT {} stage={} alignRoll={} rollOffset={} axis={} alignDelta={} safe={} align={} roll={} activeF={} activeUp={} targetF={} targetUp={} neededF={} fwdErr={}",
                mode,
                stage,
                AlignRoll,
                RollOffset.Value,
                vec(axis),
                alignDelta,
                axis.sqrMagnitude > 1e-6 && alignment_safe(axis),
                AlignmentAngle,
                RollError,
                vec(activeForward),
                vec(activeUp),
                vec(targetForward),
                vec(targetUp),
                vec(neededForward),
                forwardError);
        }

        DockingTargetFrame docking_frame => dockingResolver?.Frame;

        ModuleDockingNode active_docking_port()
        { return docking_frame?.ActivePort; }

        bool active_port_is_docking_node() => active_docking_port() != null;

        bool resolve_target()
        {
            if(dockingResolver == null || !dockingResolver.Resolve(target))
                return false;
            var frame = dockingResolver.Frame;
            targetObject = frame.TargetObject;
            targetVessel = frame.TargetVessel;
            targetTransform = frame.TargetTransform;
            dockingTarget = frame.HasTargetPort;
            approachAxis = frame.ApproachAxis;
            return targetTransform != null;
        }

        Vector3d target_velocity()
        {
            return docking_frame != null ? docking_frame.TargetVelocity :
                targetVessel != null ? targetVessel.srf_velocity : Vector3d.zero;
        }

        Vector3d target_axis()
        {
            if(docking_frame != null && docking_frame.ApproachAxis.sqrMagnitude > 1e-6)
                return docking_frame.ApproachAxis;
            if(!dockingTarget)
                return VSL.Physics.Up;
            var axis = targetObject != null ? targetObject.GetFwdVector() : Vector3.zero;
            if(axis.sqrMagnitude < 1e-6f && targetTransform != null)
                axis = targetTransform.forward;
            if(axis.sqrMagnitude < 1e-6f && targetVessel != null)
                axis = targetVessel.transform.forward;
            var axisW = ((Vector3d)axis).normalized;
            var fromVessel = (Vector3d)targetTransform.position - targetVessel.CurrentCoM;
            if(fromVessel.sqrMagnitude > 1e-4 && Vector3d.Dot(axisW, fromVessel) < 0)
                axisW = -axisW;
            if(approachAxis.sqrMagnitude > 1e-6 && Vector3d.Dot(axisW, approachAxis) < 0)
                axisW = -axisW;
            approachAxis = axisW;
            return axisW;
        }

        float hover_clearance()
        {
            var hoverAltitude = Mathf.Max(10, HoverAltitude.Value);
            if(CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain)
                return Utils.ClampL(CFG.DesiredAltitude, hoverAltitude);
            return hoverAltitude;
        }

        Vector3d overhead_point()
        {
            return (Vector3d)targetTransform.position + VSL.Physics.Up * hover_clearance();
        }

        Vector3d hold_point(Vector3d axis)
        {
            return (Vector3d)targetTransform.position + axis * HoldStandoff.Value;
        }

        Vector3d active_port_position()
        {
            if(docking_frame != null && docking_frame.ActiveTransform != null)
                return docking_frame.ActivePosition;
            return VSL.refT != null ? VSL.refT.position : VSL.Physics.wCoM;
        }

        void update_metrics(Vector3d axis)
        {
            if(poseController != null && docking_frame != null && docking_frame.Valid)
            {
                poseMetrics = poseController.Metrics(docking_frame,
                    ContactOffset.Value,
                    C.MaxLateralError,
                    C.FinalLateralFunnel,
                    HoldStandoff.Value,
                    AlignRoll,
                    RollOffset.Value);
                Distance = poseMetrics.Distance;
                LateralError = poseMetrics.LateralError;
                VerticalError = poseMetrics.VerticalError;
                AxialDistance = poseMetrics.AxialDistance;
                LateralSpeed = poseMetrics.LateralSpeed;
                ClosingSpeed = poseMetrics.ClosingSpeed;
                AlignmentAngle = poseMetrics.AlignmentAngle;
                RollError = poseMetrics.RollError;
                FinalCorridor = poseMetrics.Corridor;
                if(docking_frame.TargetPosition != Vector3d.zero)
                    VSL.Info.Destination = docking_frame.TargetPosition - VSL.Physics.wCoM;
                return;
            }
            var activePort = active_port_position();
            var targetPos = (Vector3d)targetTransform.position;
            var fromTarget = activePort - targetPos;
            var relVel = (Vector3d)VSL.vessel.srf_velocity - target_velocity();
            Distance = (float)fromTarget.magnitude;
            LateralError = (float)Vector3d.Exclude(axis, fromTarget).magnitude;
            VerticalError = (float)Vector3d.Dot(fromTarget, VSL.Physics.Up);
            AxialDistance = (float)Vector3d.Dot(fromTarget, axis) - ContactOffset.Value;
            LateralSpeed = (float)Vector3d.Exclude(axis, relVel).magnitude;
            ClosingSpeed = (float)Vector3d.Dot(relVel, -axis);
            FinalCorridor = final_corridor(AxialDistance);
            AlignmentAngle = dockingTarget
                ? Utils.Angle2(VSL.refT != null ? VSL.refT.forward : VSL.Engines.CurrentDefThrustDir, (Vector3)(-axis))
                : 0;
            if(targetPos != Vector3d.zero)
                VSL.Info.Destination = targetPos - VSL.Physics.wCoM;
        }

        float desired_agl_for_port_at(Vector3d worldPoint)
        {
            VSL.Altitude.Update();
            var pointASL = (float)VSL.Body.GetAltitude((Vector3)worldPoint);
            var comOffsetToPort = Vector3.Dot((Vector3)(active_port_position() - VSL.Physics.wCoM), VSL.Physics.Up);
            return pointASL - comOffsetToPort - VSL.Altitude.TerrainAltitude;
        }

        void set_altitude_to_world_point(Vector3d worldPoint)
        {
            if(ALT == null)
                return;
            CFG.BlockThrottle = true;
            CFG.AltitudeAboveTerrain = true;
            VSL.Altitude.Update();
            CFG.DesiredAltitude = desired_agl_for_port_at(worldPoint);
            CFG.VF.On(VFlight.AltitudeControl);
        }

        void sync_hover_altitude()
        {
            if(ALT == null)
                return;
            CFG.BlockThrottle = true;
            CFG.AltitudeAboveTerrain = true;
            VSL.Altitude.Update();
            CFG.VF.On(VFlight.AltitudeControl);
            var hoverAltitude = Mathf.Max(10, HoverAltitude.Value);
            if(CFG.DesiredAltitude < hoverAltitude + VSL.Geometry.H)
                CFG.DesiredAltitude = hoverAltitude + VSL.Geometry.H;
        }

        void set_vertical_position(float error, float maxSpeed)
        {
            CFG.BlockThrottle = true;
            CFG.VF.OffIfOn(VFlight.AltitudeControl);
            CFG.VerticalCutoff = Utils.Clamp(error * C.VerticalGain, -maxSpeed, maxSpeed);
        }

        void set_vertical_speed(float speed)
        {
            CFG.BlockThrottle = true;
            CFG.VF.OffIfOn(VFlight.AltitudeControl);
            CFG.VerticalCutoff = Utils.Clamp(speed, -VerticalSpeedControl.C.MaxSpeed, VerticalSpeedControl.C.MaxSpeed);
        }

        void set_vertical_to_point(Vector3d worldPoint, float fallbackError, float maxSpeed)
        {
            if(ALT != null)
                set_altitude_to_world_point(worldPoint);
            else
                set_vertical_position(fallbackError, maxSpeed);
        }

        void set_vertical_to_hold(Vector3d holdPoint, float verticalError, float maxSpeed)
        {
            // Close docking is relative to the target port, not terrain. AltitudeControl may
            // preserve a hover altitude and fight the last few meters of descent.
            set_vertical_position(verticalError, maxSpeed);
        }

        void set_horizontal_approach(Vector3d error, Vector3d tvel, float maxSpeed, bool noseOnCourse = true)
        {
            // NoseOnCourse engages BearingControl which yaws the nose along the needed
            // velocity. For an upward-facing docking axis that yaw IS the port roll, so
            // close-range recentering must use Move to keep the port roll undisturbed.
            CFG.HF.OnIfNot(noseOnCourse ? HFlight.NoseOnCourse : HFlight.Move);
            if(brakingController != null)
            {
                brakingController.Apply(error, tvel, maxSpeed, C.HoldTolerance);
                return;
            }
            var up = VSL.Physics.Up;
            var hError = Vector3d.Exclude(up, error);
            var hdist = hError.magnitude;
            if(hdist < 0.1)
            {
                VSL.HorizontalSpeed.SetNeeded(tvel);
                return;
            }
            var vdir = hError / hdist;
            VSL.Info.Destination = hError;
            var vdir3 = (Vector3)vdir;
            var relVel = VSL.HorizontalSpeed.Vector - tvel;
            var cur_vel = (float)Vector3d.Dot(relVel, vdir);
            var brake_dist = (float)Utils.ClampL(hdist - C.HoldTolerance, 0);
            var max_speed = maxSpeed;
            if(cur_vel > 0.1f)
            {
                var mg2 = VSL.Physics.mg * VSL.Physics.mg;
                var brake_thrust = Mathf.Min(VSL.Physics.mg, VSL.Engines.MaxThrustM / 2 * VSL.OnPlanetParams.TWRf);
                var max_thrust = Mathf.Min(Mathf.Sqrt(brake_thrust * brake_thrust + mg2),
                    VSL.Engines.MaxThrustM * 0.99f);
                var horizontal_thrust = VSL.Engines.TranslationThrustLimits.Project(VSL.LocalDir(vdir)).magnitude;
                if(horizontal_thrust > brake_thrust)
                    brake_thrust = horizontal_thrust;
                if(brake_thrust > 0)
                {
                    var brake_accel = brake_thrust / VSL.Physics.M;
                    var prep_time = 0f;
                    var brake_angle = Utils.Angle2(VSL.Engines.CurrentDefThrustDir, vdir3) - 45;
                    if(brake_angle > 0 && VSL.Torque.Slow)
                    {
                        var axis = Vector3.Cross(VSL.Engines.CurrentDefThrustDir, vdir3);
                        prep_time = VSL.Torque.NoEngines.RotationTime3Phase(brake_angle, axis,
                            PointNavigator.C.RotationAccelPhase);
                        prep_time += Utils.LerpTime(VSL.Engines.Thrust.magnitude, VSL.Engines.MaxThrustM, max_thrust,
                            VSL.Engines.AccelerationSpeed);
                    }
                    var prep_dist = cur_vel * prep_time + C.HoldTolerance;
                    var eta = brake_dist / Mathf.Max(cur_vel, 0.1f);
                    coarse_max_speed.TauUp = PointNavigator.C.MaxSpeedFilterUp / eta / brake_accel;
                    coarse_max_speed.TauDown = eta * brake_accel / PointNavigator.C.MaxSpeedFilterDown;
                    coarse_max_speed.Update(prep_dist < brake_dist
                        ? (1 + Mathf.Sqrt(1 + 2 / brake_accel * (brake_dist - prep_dist))) * brake_accel
                        : 2 * brake_accel);
                    CoarseCorrectionPID.Min = -VSL.HorizontalSpeed.Absolute;
                    if(coarse_max_speed < cur_vel)
                        CoarseCorrectionPID.Update(coarse_max_speed - cur_vel);
                    else
                    {
                        CoarseCorrectionPID.IntegralError *= (1 - TimeWarp.fixedDeltaTime * PointNavigator.C.CorrectionEasingRate);
                        CoarseCorrectionPID.Update(0);
                    }
                    if(HSC != null)
                        HSC.AddRawCorrection((Vector3)(CoarseCorrectionPID.Action * relVel.normalized));
                }
                if(coarse_max_speed < maxSpeed)
                    max_speed = Mathf.Max(HorizontalSpeedControl.C.TranslationMinDeltaV + 0.1f, coarse_max_speed);
            }
            CoarseDistancePID.Min = HorizontalSpeedControl.C.TranslationMinDeltaV + 0.1f;
            CoarseDistancePID.Max = max_speed;
            CoarseDistancePID.Update(brake_dist);
            VSL.HorizontalSpeed.SetNeeded(tvel + vdir * CoarseDistancePID.Action);
        }

        void ensure_hover_engines()
        {
            CFG.BlockThrottle = true;
            if(VSL.Engines.NoActiveEngines)
                VSL.Engines.ActivateEngines();
        }

        void keep_level()
        {
            CFG.AT.OffIfOn(Attitude.Custom);
            CFG.HF.OnIfNot(HFlight.Level);
            RollError = 0;
        }

        bool terminal_controller_enabled()
        {
            return TerminalMode != TerminalControlMode.Disabled;
        }

        void enable_rcs_for_docking(bool force = false)
        {
            if((!force && !AutoEnableRcs) || rcsEnabledByAutopilot || VSL.vessel.ActionGroups[KSPActionGroup.RCS])
                return;
            VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
            rcsEnabledByAutopilot = true;
            debug_log("RCS auto-enabled stage={} force={} terminalMode={} dist={} lateral={} axial={} transAvail={}",
                stage,
                force,
                TerminalMode,
                Distance,
                LateralError,
                AxialDistance,
                VSL.Controls.TranslationAvailable);
        }

        void restore_rcs()
        {
            if(!rcsEnabledByAutopilot)
                return;
            debug_log("RCS restored-off stage={} rcsGroupWas={}", stage, VSL.vessel.ActionGroups[KSPActionGroup.RCS]);
            VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, false);
            rcsEnabledByAutopilot = false;
        }

        void set_translation_control(Vector3d desiredVelocity, Vector3d positionError, float maxSpeed, bool includeVertical = false)
        {
            if(TRA == null)
                return;
            enable_rcs_for_docking(terminal_controller_enabled());
            var correction = positionError * C.PositionGain;
            if(correction.magnitude > maxSpeed)
                correction = correction.normalized * maxSpeed;
            desiredVelocity += correction;
            var targetVel = target_velocity();
            var relativeVelocity = desiredVelocity - targetVel;
            if(relativeVelocity.magnitude > maxSpeed)
                desiredVelocity = targetVel + relativeVelocity.normalized * maxSpeed;
            var current = (Vector3d)VSL.vessel.srf_velocity;
            var deltaV = current - desiredVelocity;
            var localDelta = VSL.LocalDir(deltaV);
            if(!includeVertical)
            {
                var upLocal = VSL.LocalDir(VSL.Physics.Up);
                localDelta -= Vector3.Project(localDelta, upLocal);
            }
            TRA.AddDeltaV(localDelta);
        }

        void set_relative_pose(Vector3d desiredPosition, Vector3d desiredRelativeVelocity, float maxSpeed, bool includeVertical = false)
        {
            if(poseController != null && docking_frame != null)
            {
                poseController.TranslateTo(docking_frame,
                    desiredPosition,
                    desiredRelativeVelocity,
                    maxSpeed,
                    includeVertical || stage == Stage.FinalApproach && Math.Abs(VerticalError) < C.MaxVerticalError);
                return;
            }
            set_translation_control(target_velocity() + desiredRelativeVelocity,
                desiredPosition - active_port_position(),
                maxSpeed,
                includeVertical);
        }

        Vector3 docking_up(Vector3 neededForward)
        {
            var neededUp = dockingTarget && targetTransform != null
                ? (Vector3)targetTransform.up
                : (Vector3)VSL.Physics.Up;
            neededUp = Vector3.ProjectOnPlane(neededUp, neededForward).normalized;
            if(neededUp.sqrMagnitude < 1e-4f)
                neededUp = Vector3.ProjectOnPlane((Vector3)VSL.Physics.Up, neededForward).normalized;
            if(Math.Abs(RollOffset.Value) > 0.01f)
                neededUp = (Quaternion.AngleAxis(RollOffset.Value, neededForward) * neededUp).normalized;
            return neededUp;
        }

        float max_terminal_tilt()
        {
            return Mathf.Min(C.MaxTerminalTilt, MaxAlignmentAngle.Value * 0.5f);
        }

        /// <summary>
        /// Desired lateral acceleration (PD on lateral offset from the approach axis)
        /// to be produced by tilting the hover attitude. This provides engine-based
        /// lateral station-keeping that works regardless of RCS layout, in vacuum
        /// and in atmosphere alike.
        /// </summary>
        Vector3d lateral_steer(Vector3d axis)
        {
            if(targetTransform == null)
                return Vector3d.zero;
            var fromTarget = active_port_position() - (Vector3d)targetTransform.position;
            var lateralError = -Vector3d.Exclude(axis, fromTarget);
            var relVel = (Vector3d)VSL.vessel.srf_velocity - target_velocity();
            var lateralVel = Vector3d.Exclude(axis, relVel);
            var accel = lateralError * C.TerminalTiltGainP - lateralVel * C.TerminalTiltGainD;
            var maxAccel = VSL.Physics.G * Mathf.Tan(max_terminal_tilt() * Mathf.Deg2Rad);
            if(maxAccel > 0 && accel.sqrMagnitude > maxAccel * maxAccel)
                accel = accel.normalized * maxAccel;
            return accel;
        }

        void set_alignment(Vector3d axis, bool alignRoll = true)
        { set_alignment(axis, alignRoll, Vector3d.zero); }

        void set_alignment(Vector3d axis, bool alignRoll, Vector3d lateralAccel)
        {
            if(!dockingTarget || ATC == null || VSL.refT == null || docking_frame == null)
            {
                debug_attitude_command("keep-level:invalid-align-context", axis);
                keep_level();
                return;
            }
            if(!alignment_safe(axis))
            {
                debug_attitude_command("keep-level:unsafe-axis", axis);
                keep_level();
                statusKey = "HoverDocking_UnsafeAlignment";
                statusFallback = "Port orientation mismatch is too large for safe hover attitude alignment.";
                return;
            }
            var useRoll = alignRoll && AlignRoll;
            CFG.HF.OffIfOn(HFlight.NoseOnCourse, HFlight.Move, HFlight.Level, HFlight.CruiseControl);
            CFG.AT.OnIfNot(Attitude.Custom);
            if(poseController != null)
            {
                debug_attitude_command(useRoll ? "pose-align" : "pose-align-forward", axis);
                poseController.Align(docking_frame, useRoll, RollOffset.Value, lateralAccel, max_terminal_tilt());
                if(useRoll)
                    RollError = poseMetrics.RollError;
                return;
            }
            var neededForward = (Vector3)(-axis).normalized;
            var curFwd = docking_frame.ActiveTransform != null ? docking_frame.ActiveTransform.forward : VSL.refT.forward;
            if(!useRoll || targetTransform == null)
            {
                if(useRoll)
                    RollError = 0;
                debug_attitude_command("align-forward-only", axis);
                ATC.SetCustomRotationW(curFwd, neededForward);
                return;
            }
            var neededUp = docking_up(neededForward);
            var curUp = VSL.refT.up;
            var forwardAligned = Utils.Angle2(curFwd, neededForward);
            if(forwardAligned < MaxAlignmentAngle.Value * 2)
            {
                var curUpProj = Vector3.ProjectOnPlane(curUp, neededForward).normalized;
                if(curUpProj.sqrMagnitude > 1e-4f)
                {
                    RollError = Utils.Angle2(curUpProj, neededUp);
                    debug_attitude_command("align-roll", axis);
                    ATC.SetCustomRotationW(curUpProj, neededUp);
                    return;
                }
            }
            RollError = Utils.Angle2(Vector3.ProjectOnPlane(curUp, neededForward), neededUp);
            debug_attitude_command("align-forward-before-roll", axis);
            ATC.SetCustomRotationW(curFwd, neededForward);
        }

        bool translation_ready()
        {
            var forceTerminalRcs = terminal_controller_enabled();
            if(AutoEnableRcs || forceTerminalRcs)
                enable_rcs_for_docking(forceTerminalRcs);
            else if(VSL.vessel.ActionGroups[KSPActionGroup.RCS] == false)
            {
                if(Time.time >= nextDebugTranslationTime)
                {
                    nextDebugTranslationTime = Time.time + 1f;
                    debug_log("TRANSLATION blocked: RCS off and AutoRCS disabled stage={} terminalMode={} TRA={} transAvail={}",
                        stage,
                        TerminalMode,
                        TRA != null,
                        VSL.Controls.TranslationAvailable);
                }
                return false;
            }
            var ready = TRA != null && VSL.Controls.TranslationAvailable;
            if(!ready && Time.time >= nextDebugTranslationTime)
            {
                nextDebugTranslationTime = Time.time + 1f;
                debug_log("TRANSLATION blocked: TRA={} transAvail={} rcsGroup={} autoRCS={} terminalMode={} stage={}",
                    TRA != null,
                    VSL.Controls.TranslationAvailable,
                    VSL.vessel.ActionGroups[KSPActionGroup.RCS],
                    AutoEnableRcs,
                    TerminalMode,
                    stage);
            }
            return ready;
        }

        bool roll_aligned(float limit)
        {
            return !AlignRoll || !dockingTarget || RollError < limit;
        }

        bool roll_aligned()
        {
            return roll_aligned(MaxAlignmentAngle.Value);
        }

        bool roll_aligned_for_close()
        {
            return roll_aligned(Mathf.Max(MaxAlignmentAngle.Value, C.MaxRollCaptureAngle));
        }

        float close_alignment_limit()
        {
            return Mathf.Max(MaxAlignmentAngle.Value, C.FinalCaptureAlignmentAngle);
        }

        void apply_docking_attitude(Vector3d axis, bool alignRoll = true)
        {
            if(dockingTarget && alignment_safe(axis))
                set_alignment(axis, alignRoll);
            else
                keep_level();
        }

        Vector3d needed_docking_forward(Vector3d axis)
        {
            if(axis.sqrMagnitude > 1e-6)
                return (-axis).normalized;
            var targetTr = docking_frame?.TargetTransform;
            if(targetTr != null && targetTr.forward.sqrMagnitude > 1e-6f)
                return -(Vector3d)targetTr.forward.normalized;
            return Vector3d.zero;
        }

        Vector3d active_docking_forward()
        {
            var activeTransform = docking_frame?.ActiveTransform;
            if(activeTransform != null)
                return (Vector3d)activeTransform.forward;
            return VSL.refT != null ? (Vector3d)VSL.refT.forward : Vector3d.zero;
        }

        float alignment_delta(Vector3d axis)
        {
            var needed = needed_docking_forward(axis);
            var current = active_docking_forward();
            if(needed.sqrMagnitude < 1e-6 || current.sqrMagnitude < 1e-6)
                return 0;
            return (float)Utils.Angle2(current, needed);
        }

        bool alignment_safe(Vector3d axis)
        {
            if(targetVessel == null || !targetVessel.LandedOrSplashed)
                return true;
            // Compare the active port to the required docking orientation, not the approach
            // axis to local up. Side/radial ports can be nearly vertical in world space while
            // still requiring a large hover attitude change from a level hold.
            return alignment_delta(axis) < C.MaxSurfaceAlignmentTilt;
        }

        float final_corridor(float axialDistance)
        {
            var finalCorridor = Utils.ClampL(C.MaxLateralError * 0.5f, 0.1f);
            var holdCorridor = Utils.ClampL(C.MaxLateralError * C.FinalLateralFunnel, finalCorridor);
            var span = Utils.ClampL(HoldStandoff.Value - ContactOffset.Value, 0.5f);
            return Mathf.Lerp(finalCorridor, holdCorridor, Utils.Clamp((float)(axialDistance / span), 0, 1));
        }

        float hold_lateral_tolerance()
        {
            return Mathf.Max(C.MaxLateralError, final_corridor(HoldStandoff.Value - ContactOffset.Value));
        }

        bool at_standoff_point(Vector3d holdPoint)
        {
            return (holdPoint - active_port_position()).magnitude < C.HoldTolerance &&
                   LateralError < hold_lateral_tolerance() &&
                   Math.Abs(AxialDistance - (HoldStandoff.Value - ContactOffset.Value)) < C.HoldTolerance &&
                   LateralSpeed < C.MaxHoldSpeed;
        }

        bool aligned_for_final_approach()
        {
            return AlignmentAngle < MaxAlignmentAngle.Value && roll_aligned();
        }

        bool stable_for_final_approach(Vector3d holdPoint)
        {
            var lateralLimit = Mathf.Min(C.MaxLateralError * 0.5f, final_corridor(HoldStandoff.Value - ContactOffset.Value) * 0.5f);
            var lateralSpeedLimit = Mathf.Min(FinalApproachSpeed.Value * 0.5f, C.MaxHoldSpeed * 0.25f);
            var closingSpeedLimit = Mathf.Max(C.FinalMinSpeed, FinalApproachSpeed.Value * 0.25f);
            return (holdPoint - active_port_position()).magnitude < C.HoldTolerance &&
                   LateralError < lateralLimit &&
                   Math.Abs(AxialDistance - (HoldStandoff.Value - ContactOffset.Value)) < C.HoldTolerance &&
                   LateralSpeed < lateralSpeedLimit &&
                   Math.Abs(ClosingSpeed) < closingSpeedLimit;
        }

        bool terminal_hold_needs_coarse_recenter()
        {
            if(TerminalMode == TerminalControlMode.RcsOnly)
                return false;
            // Coarse recenter hands the attitude over to HorizontalSpeedControl and loses
            // the port alignment, so use it only as a safety net for large excursions;
            // translation control with hover-tilt steering handles everything below this.
            var lateralLimit = 2 * Mathf.Max(hold_lateral_tolerance(), final_corridor(HoldStandoff.Value - ContactOffset.Value));
            var lateralSpeedLimit = Mathf.Max(FinalApproachSpeed.Value * 4, C.MaxHoldSpeed * 0.5f);
            return LateralError > lateralLimit || LateralSpeed > lateralSpeedLimit;
        }

        bool held_stable_for_final_approach(Vector3d holdPoint)
        {
            var stable = stable_for_final_approach(holdPoint) && aligned_for_final_approach();
            if(!stable)
            {
                finalStableSince = -1;
                return false;
            }
            if(finalStableSince < 0)
            {
                finalStableSince = Time.time;
                debug_log("FINAL hold stable started stage={} lat={} axial={} latV={} closeV={} align={} roll={}",
                    stage,
                    LateralError,
                    AxialDistance,
                    LateralSpeed,
                    ClosingSpeed,
                    AlignmentAngle,
                    RollError);
            }
            return Time.time - finalStableSince >= C.FinalAlignmentHoldTime;
        }

        float final_closing_speed(float axialDistance)
        {
            if(axialDistance <= C.FinalMinSpeed)
                return C.FinalMinSpeed;
            return Utils.Clamp(axialDistance * 0.25f, C.FinalMinSpeed, FinalApproachSpeed.Value);
        }

        bool in_terminal_atmosphere()
        {
            return VSL.Body != null
                   && VSL.Body.atmosphere
                   && VSL.Altitude.Absolute < VSL.Body.atmosphereDepth;
        }

        Vector3d terminal_lateral_direction(Vector3d axis)
        {
            var lateral = Vector3d.Cross(axis, VSL.Physics.Up);
            if(lateral.sqrMagnitude < 1e-6 && targetTransform != null)
                lateral = Vector3d.Cross(axis, (Vector3d)targetTransform.up);
            if(lateral.sqrMagnitude < 1e-6 && VSL.refT != null)
                lateral = VSL.refT.right;
            return lateral.sqrMagnitude > 1e-6 ? lateral.normalized : Vector3d.right;
        }

        float rcs_acceleration_in_direction(Vector3d worldDir)
        {
            if(worldDir.sqrMagnitude < 1e-6 || VSL.Engines.NumActiveRCS == 0 || VSL.Physics.M <= 0)
                return 0;
            var localDir = VSL.LocalDir((Vector3)worldDir.normalized).normalized;
            return VSL.Engines.MaxThrustRCS.Project(localDir).magnitude / VSL.Physics.M;
        }

        bool terminal_rcs_authority(Vector3d axis, out string reason)
        {
            reason = "ok";
            if(AutoEnableRcs)
                enable_rcs_for_docking();
            else if(terminal_controller_enabled())
                enable_rcs_for_docking(true);
            if(TRA == null)
            {
                reason = "no translation controller";
                return false;
            }
            if(!VSL.vessel.ActionGroups[KSPActionGroup.RCS])
            {
                reason = "RCS action group is off";
                return false;
            }
            if(VSL.Engines.NumActiveRCS == 0)
            {
                reason = "no active RCS thrusters";
                return false;
            }
            var upAccel = rcs_acceleration_in_direction(VSL.Physics.Up);
            var downAccel = rcs_acceleration_in_direction(-VSL.Physics.Up);
            var axisAccel = rcs_acceleration_in_direction(axis);
            var antiAxisAccel = rcs_acceleration_in_direction(-axis);
            var lateral = terminal_lateral_direction(axis);
            var lateralAccel = rcs_acceleration_in_direction(lateral);
            var antiLateralAccel = rcs_acceleration_in_direction(-lateral);
            var supportOK = VSL.Physics.G < C.TerminalRcsLowGravity
                            || upAccel > VSL.Physics.G * C.TerminalRcsMinTWR;
            var controlOK = axisAccel > C.TerminalRcsMinControlAccel
                            && antiAxisAccel > C.TerminalRcsMinControlAccel
                            && lateralAccel > C.TerminalRcsMinControlAccel
                            && antiLateralAccel > C.TerminalRcsMinControlAccel
                            && downAccel > C.TerminalRcsMinControlAccel;
            reason = Utils.Format("support={} control={} g={} upA={} downA={} axisA={} antiAxisA={} latA={} antiLatA={} minA={} minTWR={} atmosphere={}",
                supportOK,
                controlOK,
                VSL.Physics.G,
                upAccel,
                downAccel,
                axisAccel,
                antiAxisAccel,
                lateralAccel,
                antiLateralAccel,
                C.TerminalRcsMinControlAccel,
                C.TerminalRcsMinTWR,
                in_terminal_atmosphere());
            return supportOK && controlOK;
        }

        TerminalControlMode select_terminal_mode(Vector3d axis)
        {
            var selected = TerminalControlMode.HoverAssist;
            var reason = "manual HoverAssist";
            if(TerminalMode == TerminalControlMode.Disabled)
            {
                selected = TerminalControlMode.Disabled;
                reason = "terminal controller disabled";
            }
            else if(TerminalMode == TerminalControlMode.HoverAssist)
            {
                selected = TerminalControlMode.HoverAssist;
            }
            else
            {
                string authorityReason;
                var authorityOK = terminal_rcs_authority(axis, out authorityReason);
                var autoAllowsRcsOnly = TerminalMode != TerminalControlMode.Auto || !in_terminal_atmosphere();
                if(authorityOK && autoAllowsRcsOnly)
                {
                    selected = TerminalControlMode.RcsOnly;
                    reason = "RCS-only allowed: " + authorityReason;
                }
                else
                {
                    selected = TerminalControlMode.HoverAssist;
                    reason = (TerminalMode == TerminalControlMode.RcsOnly ? "RCS-only fallback: " : "Auto chose HoverAssist: ")
                             + (authorityOK ? "atmosphere present" : authorityReason);
                }
            }
            if(resolvedTerminalMode != selected || Time.time >= nextDebugTerminalModeTime)
            {
                nextDebugTerminalModeTime = Time.time + 2f;
                debug_log("TERMINAL mode request={} selected={} reason={}",
                    TerminalMode,
                    selected,
                    reason);
            }
            resolvedTerminalMode = selected;
            return selected;
        }

        void set_main_throttle(float throttle)
        {
            VSL.vessel.ctrlState.mainThrottle = throttle;
            if(VSL.IsActiveVessel)
                FlightInputHandler.state.mainThrottle = throttle;
        }

        void set_rcs_only_hover_support()
        {
            CFG.VF.OffIfOn(VFlight.AltitudeControl);
            CFG.VerticalCutoff = VerticalSpeedControl.C.MaxSpeed;
            CFG.BlockThrottle = false;
            set_main_throttle(0);
            if(rcsOnlyTerminalActive)
                return;
            rcsOnlyTerminalActive = true;
            debug_log("TERMINAL RCS-only engaged stage={} lat={} axial={} closeV={} throttle=0",
                stage,
                LateralError,
                AxialDistance,
                ClosingSpeed);
        }

        void restore_hover_support()
        {
            if(freeFallTerminalActive)
            {
                freeFallTerminalActive = false;
                CFG.BlockThrottle = true;
                CFG.VerticalCutoff = 0;
                debug_log("TERMINAL free-fall ended stage={} lat={} axial={} closeV={}",
                    stage,
                    LateralError,
                    AxialDistance,
                    ClosingSpeed);
            }
            if(!rcsOnlyTerminalActive)
                return;
            rcsOnlyTerminalActive = false;
            CFG.BlockThrottle = true;
            CFG.VerticalCutoff = 0;
            debug_log("TERMINAL hover support restored stage={} lat={} axial={} closeV={}",
                stage,
                LateralError,
                AxialDistance,
                ClosingSpeed);
        }

        bool gravity_aids_closing(Vector3d axis)
        {
            if(VSL.Physics.G < C.TerminalRcsLowGravity)
                return false;
            // Closing moves along -axis; free fall helps when that direction has a
            // downward component (typical hover-docking from above a landed target).
            return Vector3d.Dot(VSL.Physics.Up, axis) > 0.05;
        }

        bool aligned_for_free_fall(float corridor)
        {
            return AlignmentAngle < close_alignment_limit()
                   && roll_aligned_for_close()
                   && LateralError < Mathf.Max(corridor * 2f, C.MaxLateralError * 2f);
        }

        bool body_has_atmosphere()
        {
            return VSL.Body != null && VSL.Body.atmosphere;
        }

        float effective_free_fall_distance()
        {
            return body_has_atmosphere()
                ? FreeFallDistanceAtmosphere.Value
                : FreeFallDistanceVacuum.Value;
        }

        bool in_free_fall_zone(float axialDistance)
        {
            var freeFall = effective_free_fall_distance();
            return freeFall > 0 && axialDistance <= freeFall;
        }

        void maintain_free_fall_support()
        {
            CFG.VF.OffIfOn(VFlight.AltitudeControl);
            CFG.VerticalCutoff = 0;
            CFG.BlockThrottle = false;
            set_main_throttle(0);
            VSL.HorizontalSpeed.SetNeeded(target_velocity());
            CFG.HF.OffIfOn(HFlight.NoseOnCourse, HFlight.Move, HFlight.Level, HFlight.CruiseControl);
        }

        void set_free_fall_support()
        {
            if(rcsOnlyTerminalActive)
                rcsOnlyTerminalActive = false;
            maintain_free_fall_support();
            if(freeFallTerminalActive)
                return;
            freeFallTerminalActive = true;
            debug_log("TERMINAL free-fall engaged stage={} lat={} axial={} align={} roll={} freeFall={} atmo={} g={}",
                stage,
                LateralError,
                AxialDistance,
                AlignmentAngle,
                RollError,
                effective_free_fall_distance(),
                body_has_atmosphere(),
                VSL.Physics.G);
        }

        void execute_free_fall_docking(Vector3d axis)
        {
            set_alignment(axis, true);
            CFG.HF.OffIfOn(HFlight.NoseOnCourse, HFlight.Move, HFlight.Level, HFlight.CruiseControl);
            set_free_fall_support();
            statusKey = "HoverDocking_StatusFreeFall";
            statusFallback = "Ports aligned; allowing gravity settle for final capture.";
        }

        bool terminal_context_valid(Vector3d axis)
        {
            if(!alignment_safe(axis))
            {
                stage = Stage.HoldAndAlign;
                statusKey = "HoverDocking_UnsafeAlignment";
                statusFallback = "Port orientation mismatch is too large for safe hover attitude alignment.";
                return false;
            }
            if(!translation_ready())
            {
                stage = Stage.HoldAndAlign;
                statusKey = "HoverDocking_NoTranslation";
                statusFallback = "Holding at standoff; translation control is required for final docking.";
                return false;
            }
            if(!dockingTarget)
            {
                stage = Stage.HoldAndAlign;
                return false;
            }
            if(!active_port_is_docking_node())
            {
                stage = Stage.HoldAndAlign;
                statusKey = "HoverDocking_NoActivePort";
                statusFallback = "Right-click your docking port and select Control From Here.";
                return false;
            }
            return true;
        }

        void execute_terminal_approach(Vector3d axis, bool terminalController)
        {
            if(!terminal_context_valid(axis))
            {
                restore_hover_support();
                return;
            }
            set_alignment(axis, true, lateral_steer(axis));
            CFG.HF.OffIfOn(HFlight.NoseOnCourse, HFlight.Move, HFlight.Level, HFlight.CruiseControl);

            var rcsOnly = terminalController && resolvedTerminalMode == TerminalControlMode.RcsOnly;
            if(rcsOnly)
            {
                string authorityReason;
                if(!terminal_rcs_authority(axis, out authorityReason))
                {
                    restore_hover_support();
                    resolvedTerminalMode = TerminalControlMode.HoverAssist;
                    finalStableSince = -1;
                    stage = Stage.TerminalHold;
                    statusKey = "HoverDocking_RcsOnlyUnavailable";
                    statusFallback = "RCS-only terminal control is unavailable; restoring hover assist.";
                    debug_log("TERMINAL RCS-only authority lost: {}", authorityReason);
                    return;
                }
                set_rcs_only_hover_support();
            }
            else
                restore_hover_support();

            var activeFromTarget = active_port_position() - (Vector3d)targetTransform.position;
            var axialDistance = (float)Vector3d.Dot(activeFromTarget, axis) - ContactOffset.Value;
            var corridor = final_corridor(axialDistance);
            var inFreeFallZone = in_free_fall_zone(axialDistance);
            if(inFreeFallZone && aligned_for_free_fall(corridor) && gravity_aids_closing(axis))
            {
                execute_free_fall_docking(axis);
                return;
            }
            if(freeFallTerminalActive)
                restore_hover_support();
            var recenterLateralError = Mathf.Max(corridor * C.TerminalCorridorHysteresis, C.MaxLateralError);
            var recenterLateralSpeed = Mathf.Max(FinalApproachSpeed.Value * 2, C.MaxHoldSpeed * 0.25f);
            var severeLateralError = Mathf.Max(corridor * C.TerminalSevereDriftFactor,
                C.MaxLateralError * C.TerminalSevereDriftFactor);
            var unsafeClosingSpeed = ClosingSpeed > Mathf.Max(C.MaxFinalSpeed * 2, FinalApproachSpeed.Value * 4);
            // Within capture range a full retreat throws away an almost complete approach
            // and restarts the whole stability countdown; mild drift near contact is
            // handled by the pause-and-recenter branch below instead.
            var closeRange = axialDistance < C.HoldTolerance;
            var needsTerminalRecenter = terminalController
                                        && !closeRange
                                        && !inFreeFallZone
                                        && (LateralError > recenterLateralError
                                            || LateralSpeed > recenterLateralSpeed);
            var severeFactor = inFreeFallZone ? 2f : 1f;
            var alignFactor = inFreeFallZone ? 4f : 2f;
            if(needsTerminalRecenter
               || LateralError > severeLateralError * severeFactor
               || AlignmentAngle > close_alignment_limit() * alignFactor
               || axialDistance > HoldStandoff.Value * 2
               || unsafeClosingSpeed)
            {
                restore_hover_support();
                finalStableSince = -1;
                stage = terminalController ? Stage.TerminalHold : Stage.HoldAndAlign;
                statusKey = "HoverDocking_StatusTerminalBackout";
                statusFallback = "Final drift is too large; backing out to standoff.";
                debug_log("TERMINAL backout lat={} recenterLat={} severeLat={} latV={} recenterLatV={} align={} axial={} closeV={} recenter={} unsafeClose={}",
                    LateralError,
                    recenterLateralError,
                    severeLateralError,
                    LateralSpeed,
                    recenterLateralSpeed,
                    AlignmentAngle,
                    axialDistance,
                    ClosingSpeed,
                    needsTerminalRecenter,
                    unsafeClosingSpeed);
                return;
            }
            var mildDrift = LateralError > corridor || LateralSpeed > FinalApproachSpeed.Value;
            var tooFastToClose = ClosingSpeed > FinalApproachSpeed.Value * 1.25f;
            var canClose = !tooFastToClose
                           && LateralError < corridor
                           && LateralSpeed < FinalApproachSpeed.Value
                           && AlignmentAngle < close_alignment_limit()
                           && roll_aligned_for_close();
            var speed = canClose ? final_closing_speed(axialDistance) : 0;
            var currentAxial = (float)Vector3d.Dot(activeFromTarget, axis);
            var desiredAxial = ContactOffset.Value;
            if(!canClose)
            {
                if(Time.time >= nextDebugFinalHoldTime)
                {
                    nextDebugFinalHoldTime = Time.time + 0.5f;
                    debug_log("TERMINAL pause-recenter mode={} lat={} corridor={} hysteresis={} latV={} closeV={} tooFast={} mildDrift={} align={} roll={} axial={}",
                        resolvedTerminalMode,
                        LateralError,
                        corridor,
                        corridor * C.TerminalCorridorHysteresis,
                        LateralSpeed,
                        ClosingSpeed,
                        tooFastToClose,
                        mildDrift,
                        AlignmentAngle,
                        RollError,
                        axialDistance);
                }
                desiredAxial = Utils.Clamp(currentAxial, ContactOffset.Value, HoldStandoff.Value);
                if(tooFastToClose)
                {
                    var brakeOffset = Utils.Clamp((ClosingSpeed - FinalApproachSpeed.Value) * 0.5f, 0, C.HoldTolerance);
                    desiredAxial = Utils.Clamp(desiredAxial + brakeOffset, ContactOffset.Value, HoldStandoff.Value);
                }
            }
            // The feed-forward closing speed is the only axial driver: the translation
            // target rides at the current axial distance so the position correction and
            // the closing speed do not stack up and overshoot the commanded approach speed.
            var lateralAxial = Utils.Clamp(currentAxial, desiredAxial, HoldStandoff.Value);
            var lateralPoint = (Vector3d)targetTransform.position + axis * lateralAxial;
            if(!rcsOnly)
            {
                // When closing, aim the vertical channel past the contact point so the
                // ports actually reach magnetic capture range, and never let it descend
                // faster than the closing-speed profile (MaxHoldSpeed here caused the
                // permanent "too fast" pause/resume sawtooth).
                var verticalAxial = canClose ? 0f : desiredAxial;
                var verticalPoint = (Vector3d)targetTransform.position + axis * verticalAxial;
                var verticalMax = canClose
                    ? Mathf.Max(speed, C.FinalMinSpeed)
                    : Mathf.Max(FinalApproachSpeed.Value * 0.5f, C.FinalMinSpeed);
                set_vertical_position((float)Vector3d.Dot(verticalPoint - active_port_position(), VSL.Physics.Up),
                    verticalMax);
            }
            set_relative_pose(lateralPoint, -axis * speed, C.MaxFinalSpeed, rcsOnly);
            if(canClose)
            {
                statusKey = terminalController ? "HoverDocking_StatusTerminalApproach" : "HoverDocking_StatusFinal";
                statusFallback = terminalController ? "Terminal docking approach." : "Final docking approach.";
            }
            else
            {
                statusKey = "HoverDocking_StatusTerminalCorrection";
                statusFallback = "Pausing closing and re-centering in the final corridor.";
            }
        }

        void set_cps_exemption(bool active)
        {
            var wasExempt = finalCpsExemption;
            if(CPS != null)
                CPS.SetExemptTarget(this, active ? targetVessel : null);
            finalCpsExemption = active && CPS != null && CFG.UseCPS && targetVessel != null;
            if(wasExempt != finalCpsExemption)
                debug_log("CPS exemption {} -> {} requested={} useCPS={} hasCPS={} target={}",
                    wasExempt,
                    finalCpsExemption,
                    active,
                    CFG.UseCPS,
                    CPS != null,
                    targetVessel != null ? targetVessel.vesselName : "none");
        }

        bool should_exempt_docking_target_from_cps()
        {
            if(targetVessel == null)
                return false;
            switch(stage)
            {
            case Stage.Takeoff:
            case Stage.CoarseApproach:
            case Stage.Standoff:
            case Stage.HoldAndAlign:
            case Stage.Align:
            case Stage.TerminalHold:
            case Stage.TerminalApproach:
            case Stage.FinalApproach:
                return true;
            default:
                return false;
            }
        }

        string cps_exemption_status()
        {
            if(!CFG.UseCPS)
                return Loc.T("HoverDocking_CpsDisabled", "Disabled");
            return TargetCpsExempt
                ? Loc.T("HoverDocking_CpsExemptOn", "Exempt")
                : Loc.T("HoverDocking_CpsExemptOff", "No exempt");
        }

        void hold_with_translation(Vector3d holdPoint, Vector3d axis, bool align)
        {
            CFG.HF.OffIfOn(HFlight.NoseOnCourse, HFlight.Move, HFlight.Level, HFlight.CruiseControl);
            var error = holdPoint - active_port_position();
            set_vertical_to_hold(holdPoint, (float)Vector3d.Dot(error, VSL.Physics.Up), C.MaxHoldSpeed);
            if(align && dockingTarget)
                set_alignment(axis, true, lateral_steer(axis));
            else
                keep_level();
            set_relative_pose(holdPoint, Vector3d.zero, C.MaxHoldSpeed);
        }

        void hold_with_horizontal_speed(Vector3d holdPoint)
        {
            // HorizontalSpeedControl owns the attitude while it drives horizontal velocity;
            // commanding a Custom docking attitude at the same time makes the two attitude
            // controllers fight and the vessel oscillates in roll and drifts laterally.
            keep_level();
            var error = holdPoint - active_port_position();
            set_vertical_to_hold(holdPoint, (float)Vector3d.Dot(error, VSL.Physics.Up), C.TakeoffVerticalSpeed);
            set_horizontal_approach(error, target_velocity(), C.MaxHoldSpeed, noseOnCourse: false);
        }

        protected override void Update()
        {
            if(CFG.Nav.Paused)
                return;
            if(cleanupInProgress)
                return;
            if(stage == Stage.Aborted || stage == Stage.Finished || stage == Stage.Docked)
            {
                if(CFG.Nav[Navigation.HoverDocking])
                    CFG.Nav.OffIfOn(Navigation.HoverDocking);
                return;
            }
            if(!resolve_target())
            {
                abort("HoverDocking_InvalidTarget", "Docking target is no longer valid.");
                return;
            }
            if(targetVessel == VSL.vessel)
            {
                finish(true);
                return;
            }
            if(!targetVessel.LandedOrSplashed && targetVessel.srf_velocity.magnitude > MaxTargetSpeed.Value)
            {
                abort("HoverDocking_TargetMoving", "Target is moving too fast for hover docking.");
                return;
            }
            if(freeFallTerminalActive)
                maintain_free_fall_support();
            else
            {
                ensure_hover_engines();
                if(VSL.Engines.NoActiveEngines)
                {
                    abort("HoverDocking_NoEngines", "No active engines for hover docking.");
                    return;
                }
            }

            var axis = target_axis();
            if(axis.sqrMagnitude < 1e-6)
            {
                abort("HoverDocking_InvalidAxis", "Unable to resolve docking approach axis.");
                return;
            }
            update_metrics(axis);
            debug_snapshot("tick", axis);
            var stageBefore = stage;
            if(Distance > C.AbortDistance &&
               (stage == Stage.Standoff
                || stage == Stage.Align
                || stage == Stage.HoldAndAlign
                || stage == Stage.TerminalHold
                || stage == Stage.TerminalApproach
                || stage == Stage.FinalApproach) &&
               (active_port_position() - overhead_point()).magnitude > C.AbortDistance)
            {
                abort("HoverDocking_DriftAbort", "Docking target drifted outside the safe approach corridor.");
                return;
            }

            switch(stage)
            {
            case Stage.None:
            case Stage.ResolveTarget:
                stage = VSL.LandedOrSplashed ? Stage.Takeoff : Stage.CoarseApproach;
                statusKey = "HoverDocking_StatusCoarse";
                statusFallback = "Approaching docking target.";
                break;

            case Stage.Takeoff:
                VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
                apply_docking_attitude(axis);
                sync_hover_altitude();
                if(ALT == null)
                    set_vertical_speed(C.TakeoffVerticalSpeed);
                statusKey = "HoverDocking_StatusTakeoff";
                statusFallback = "Taking off to hover.";
                if(!VSL.LandedOrSplashed && VSL.Altitude.Relative > hover_clearance() * 0.8f)
                    stage = Stage.CoarseApproach;
                break;

            case Stage.CoarseApproach:
            {
                var hoverPoint = overhead_point();
                var error = hoverPoint - active_port_position();
                var horizontalError = Vector3d.Exclude(VSL.Physics.Up, error).magnitude;
                var verticalError = Math.Abs(Vector3d.Dot(error, VSL.Physics.Up));
                sync_hover_altitude();
                if(ALT == null)
                    set_vertical_to_point(hoverPoint, (float)Vector3d.Dot(error, VSL.Physics.Up), C.TakeoffVerticalSpeed);
                apply_docking_attitude(axis);
                set_horizontal_approach(error, target_velocity(), MaxCoarseSpeed.Value);
                statusKey = "HoverDocking_StatusOverhead";
                statusFallback = "Approaching hover point above target.";
                if(horizontalError < C.HoldTolerance * 2 && verticalError < C.HoldTolerance * 2)
                    stage = Stage.Standoff;
                break;
            }

            case Stage.HoldAndAlign:
                stage = Stage.Standoff;
                break;

            case Stage.Standoff:
            {
                var holdPoint = hold_point(axis);
                if(translation_ready())
                {
                    hold_with_translation(holdPoint, axis, false);
                    if(!dockingTarget)
                    {
                        statusKey = "HoverDocking_NoDockingPort";
                        statusFallback = "Target has no docking port; cannot complete final approach.";
                    }
                    else if(!active_port_is_docking_node())
                    {
                        statusKey = "HoverDocking_NoActivePort";
                        statusFallback = "Right-click your docking port and select Control From Here.";
                    }
                    else if(at_standoff_point(holdPoint))
                    {
                        if(alignment_safe(axis))
                        {
                            stage = Stage.Align;
                            statusKey = "HoverDocking_StatusAlign";
                            statusFallback = "Holding and aligning docking ports.";
                        }
                        else
                        {
                            keep_level();
                            statusKey = "HoverDocking_UnsafeAlignment";
                            statusFallback = "Port orientation mismatch is too large for safe hover attitude alignment.";
                        }
                    }
                    else
                    {
                        statusKey = "HoverDocking_StatusStandoff";
                        statusFallback = "Moving to docking standoff point.";
                    }
                }
                else
                {
                    hold_with_horizontal_speed(holdPoint);
                    statusKey = "HoverDocking_NoTranslation";
                    statusFallback = "Holding above target; translation control is required for final docking.";
                }
                break;
            }

            case Stage.Align:
            {
                var holdPoint = hold_point(axis);
                if(!alignment_safe(axis))
                {
                    hold_with_translation(holdPoint, axis, false);
                    statusKey = "HoverDocking_UnsafeAlignment";
                    statusFallback = "Port orientation mismatch is too large for safe hover attitude alignment.";
                    break;
                }
                if(translation_ready())
                {
                    hold_with_translation(holdPoint, axis, true);
                    if(!dockingTarget)
                    {
                        statusKey = "HoverDocking_NoDockingPort";
                        statusFallback = "Target has no docking port; cannot complete final approach.";
                    }
                    else if(!active_port_is_docking_node())
                    {
                        statusKey = "HoverDocking_NoActivePort";
                        statusFallback = "Right-click your docking port and select Control From Here.";
                    }
                    else if(aligned_for_final_approach())
                    {
                        stage = Stage.TerminalHold;
                        statusKey = "HoverDocking_StatusTerminalHold";
                        statusFallback = "Docking ports aligned; holding terminal standoff.";
                    }
                    else
                    {
                        statusKey = "HoverDocking_StatusAlign";
                        statusFallback = "Holding and aligning docking ports.";
                    }
                }
                else
                {
                    hold_with_horizontal_speed(holdPoint);
                    statusKey = "HoverDocking_NoTranslation";
                    statusFallback = "Holding above target; translation control is required for final docking.";
                }
                break;
            }

            case Stage.TerminalHold:
            {
                var holdPoint = hold_point(axis);
                if(!terminal_context_valid(axis))
                {
                    break;
                }
                restore_hover_support();
                if(terminal_hold_needs_coarse_recenter())
                {
                    finalStableSince = -1;
                    hold_with_horizontal_speed(holdPoint);
                    statusKey = "HoverDocking_StatusTerminalRecenter";
                    statusFallback = "Re-centering at terminal standoff before fine alignment.";
                    if(Time.time >= nextDebugFinalHoldTime)
                    {
                        nextDebugFinalHoldTime = Time.time + 0.5f;
                        debug_log("TERMINAL hold coarse-recenter lat={} latV={} axial={} closeV={} align={} roll={} attCmd=level terminalMode={}",
                            LateralError,
                            LateralSpeed,
                            AxialDistance,
                            ClosingSpeed,
                            AlignmentAngle,
                            RollError,
                            TerminalMode);
                    }
                    break;
                }
                hold_with_translation(holdPoint, axis, true);
                if(held_stable_for_final_approach(holdPoint))
                {
                    debug_log("TERMINAL final roll check passed align={} roll={} alignLimit={} rollLimit={} holdTime={}s",
                        AlignmentAngle,
                        RollError,
                        MaxAlignmentAngle.Value,
                        Mathf.Max(MaxAlignmentAngle.Value, C.MaxRollCaptureAngle),
                        C.FinalAlignmentHoldTime);
                    var mode = select_terminal_mode(axis);
                    if(mode == TerminalControlMode.Disabled)
                    {
                        stage = Stage.FinalApproach;
                        statusKey = "HoverDocking_StatusFinal";
                        statusFallback = "Final docking approach.";
                    }
                    else
                    {
                        stage = Stage.TerminalApproach;
                        statusKey = "HoverDocking_StatusTerminalApproach";
                        statusFallback = "Terminal docking approach.";
                    }
                }
                else if(aligned_for_final_approach())
                {
                    statusKey = "HoverDocking_StatusTerminalHold";
                    statusFallback = "Docking ports aligned; holding terminal standoff.";
                }
                else
                {
                    statusKey = "HoverDocking_StatusAlign";
                    statusFallback = "Holding and aligning docking ports.";
                }
                break;
            }

            case Stage.TerminalApproach:
            {
                execute_terminal_approach(axis, true);
                break;
            }

            case Stage.FinalApproach:
            {
                execute_terminal_approach(axis, false);
                break;
            }
            }
            debug_stage_transition(stageBefore, stage, axis);
            set_cps_exemption(should_exempt_docking_target_from_cps());
        }

        string stage_label()
        {
            switch(stage)
            {
            case Stage.ResolveTarget: return Loc.T("HoverDocking_StageResolve", "Resolve target");
            case Stage.Takeoff: return Loc.T("HoverDocking_StageTakeoff", "Takeoff");
            case Stage.CoarseApproach: return Loc.T("HoverDocking_StageCoarse", "Coarse approach");
            case Stage.Standoff: return Loc.T("HoverDocking_StageStandoff", "Standoff");
            case Stage.Align: return Loc.T("HoverDocking_StageAlign", "Align");
            case Stage.HoldAndAlign: return Loc.T("HoverDocking_StageHold", "Hold and align");
            case Stage.TerminalHold: return Loc.T("HoverDocking_StageTerminalHold", "Terminal hold");
            case Stage.TerminalApproach: return Loc.T("HoverDocking_StageTerminalApproach", "Terminal approach");
            case Stage.FinalApproach: return Loc.T("HoverDocking_StageFinal", "Final approach");
            case Stage.Docked: return Loc.T("HoverDocking_StageDocked", "Docked");
            case Stage.Aborted: return Loc.T("HoverDocking_StageAborted", "Aborted");
            default: return Loc.T("HoverDocking_StageIdle", "Idle");
            }
        }

        public void DrawOptions()
        {
            var updated = false;
            GUILayout.BeginVertical();
            updated |= draw_terminal_mode_option();
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsHoverAltitude", "Hover Height:",
                    "HoverDocking_OptionsHoverAltitude_Tooltip",
                    "Altitude above the target for the overhead approach hover point. Minimum 10 m."),
                HoverAltitude, "m", 5, "F0");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsHoldStandoff", "Standoff Distance:",
                    "HoverDocking_OptionsHoldStandoff_Tooltip",
                    "Distance from the target along the docking axis while holding and aligning."),
                HoldStandoff, "m", 1, "F1");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsMaxCoarseSpeed", "Max Approach Speed:",
                    "HoverDocking_OptionsMaxCoarseSpeed_Tooltip",
                    "Maximum horizontal speed when flying to the overhead hover point."),
                MaxCoarseSpeed, "m/s", 5, "F1");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsFinalApproachSpeed", "Final Dock Speed:",
                    "HoverDocking_OptionsFinalApproachSpeed_Tooltip",
                    "Maximum closing speed during the final docking translation."),
                FinalApproachSpeed, "m/s", 0.05f, "F2");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsMaxTargetSpeed", "Max Target Speed:",
                    "HoverDocking_OptionsMaxTargetSpeed_Tooltip",
                    "Target surface speed must stay below this value or hover docking aborts."),
                MaxTargetSpeed, "m/s", 0.5f, "F1");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsMaxAlignmentAngle", "Max Alignment Angle:",
                    "HoverDocking_OptionsMaxAlignmentAngle_Tooltip",
                    "Maximum docking port misalignment allowed before final approach."),
                MaxAlignmentAngle, "\u00B0", 1, "F0");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsContactOffset", "Contact Distance:",
                    "HoverDocking_OptionsContactOffset_Tooltip",
                    "Target axial distance between docking ports at contact."),
                ContactOffset, "m", 0.1f, "F2");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsFreeFallDistanceAtmosphere", "Free Fall (Atmosphere):",
                    "HoverDocking_OptionsFreeFallDistanceAtmosphere_Tooltip",
                    "On bodies with atmosphere: within this axial distance, once ports are aligned, hover thrust is cut for gravity settle. Set to 0 to disable."),
                FreeFallDistanceAtmosphere, "m", 0.5f, "F1");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsFreeFallDistanceVacuum", "Free Fall (Vacuum):",
                    "HoverDocking_OptionsFreeFallDistanceVacuum_Tooltip",
                    "On airless bodies: within this axial distance, once ports are aligned, hover thrust is cut for gravity settle. Set to 0 to disable."),
                FreeFallDistanceVacuum, "m", 0.5f, "F1");
            updated |= draw_float_option(
                Loc.Content("HoverDocking_OptionsRollOffset", "Roll Offset:",
                    "HoverDocking_OptionsRollOffset_Tooltip",
                    "Extra roll angle applied around the docking axis, similar to MechJeb docking roll."),
                RollOffset, "\u00B0", 5, "F0");
            GUILayout.BeginHorizontal();
            var alignRoll = AlignRoll;
            var autoRcs = AutoEnableRcs;
            Utils.ButtonSwitch(Loc.T("HoverDocking_OptionsAlignRoll", "Align Roll"),
                ref AlignRoll,
                Loc.T("HoverDocking_OptionsAlignRoll_Tooltip", "Match docking port roll to the target port before final approach."),
                GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("HoverDocking_OptionsAutoRcs", "Auto RCS"),
                ref AutoEnableRcs,
                Loc.T("HoverDocking_OptionsAutoRcs_Tooltip", "Automatically enable RCS for translation during hold and final approach."),
                GUILayout.ExpandWidth(true));
            updated |= alignRoll != AlignRoll || autoRcs != AutoEnableRcs;
            GUILayout.FlexibleSpace();
            if(CFG.Nav[Navigation.HoverDocking])
            {
                if(GUILayout.Button(Loc.Content("Abort", "Abort", "HoverDocking_Abort_Tooltip", "Stop hover docking."),
                    Styles.danger_button,
                    GUILayout.Width(60)))
                    abort("HoverDocking_UserAbort", "Hover docking aborted.");
            }
            else
            {
                if(GUILayout.Button(Loc.Content("Start", "Start", "HoverDocking_Start_Tooltip", "Start hover docking with the current settings."),
                    Styles.enabled_button,
                    GUILayout.Width(60)))
                    VSL.Engines.ActivateEnginesAndRun(() => CFG.Nav.XOn(Navigation.HoverDocking));
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
            if(updated)
                SaveToConfig();
        }

        bool draw_terminal_mode_option()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.Content("HoverDocking_OptionsTerminalMode", "Terminal Mode:",
                "HoverDocking_OptionsTerminalMode_Tooltip",
                "How Hover Docking controls the final meters before docking."),
                GUILayout.Width(125));
            var changed = GUILayout.Button(terminal_mode_content(TerminalMode), GUILayout.ExpandWidth(true));
            if(changed)
                TerminalMode = next_terminal_mode(TerminalMode);
            GUILayout.EndHorizontal();
            return changed;
        }

        static GUIContent terminal_mode_content(TerminalControlMode mode)
        {
            switch(mode)
            {
            case TerminalControlMode.HoverAssist:
                return Loc.Content("HoverDocking_TerminalModeHoverAssist", "Hover Assist",
                    "HoverDocking_TerminalModeHoverAssist_Tooltip",
                    "Use hover engines for vertical support while RCS performs fine terminal translation.");
            case TerminalControlMode.RcsOnly:
                return Loc.Content("HoverDocking_TerminalModeRcsOnly", "RCS Only",
                    "HoverDocking_TerminalModeRcsOnly_Tooltip",
                    "Use RCS for full terminal control only if it can support gravity and correct the docking axes.");
            case TerminalControlMode.Disabled:
                return Loc.Content("HoverDocking_TerminalModeDisabled", "Disabled",
                    "HoverDocking_TerminalModeDisabled_Tooltip",
                    "Use the previous engine-assisted final approach behavior.");
            default:
                return Loc.Content("HoverDocking_TerminalModeAuto", "Auto",
                    "HoverDocking_TerminalModeAuto_Tooltip",
                    "Choose Hover Assist or RCS Only from atmosphere, gravity, and RCS authority.");
            }
        }

        static TerminalControlMode next_terminal_mode(TerminalControlMode mode)
        {
            switch(mode)
            {
            case TerminalControlMode.Auto: return TerminalControlMode.HoverAssist;
            case TerminalControlMode.HoverAssist: return TerminalControlMode.RcsOnly;
            case TerminalControlMode.RcsOnly: return TerminalControlMode.Disabled;
            default: return TerminalControlMode.Auto;
            }
        }

        static bool draw_float_option(GUIContent label, FloatField field, string suffix, float step, string format)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(125));
            var changed = field.Draw(suffix, step, format, suffix_width: 25);
            GUILayout.EndHorizontal();
            return changed;
        }

        public override void Draw()
        {
            if(stage != Stage.None && stage != Stage.Finished)
            {
                GUILayout.BeginVertical(Styles.white);
                GUILayout.Label(Loc.T(statusKey, statusFallback), Styles.boxed_label, GUILayout.ExpandWidth(true));
                GUILayout.Label(Loc.F("HoverDocking_StatusLine",
                                      "<<1>> | D <<2>> | Lat <<3>> | Ax <<4>> | V <<5>> | Align <<6>> | Roll <<7>> | <<9>> <<8>> | Vert <<10>> | LatV <<11>>",
                                      stage_label(),
                                      Utils.formatBigValue(Distance, "m"),
                                      Utils.formatBigValue(LateralError, "m"),
                                      Utils.formatBigValue(AxialDistance, "m"),
                                      Utils.formatBigValue(ClosingSpeed, "m/s"),
                                      AlignmentAngle.ToString("F1") + "?",
                                      RollError.ToString("F1") + "?",
                                      cps_exemption_status(),
                                      Loc.T("HoverDocking_CpsExemptLabel", "CPS exempt"),
                                      Utils.formatBigValue(VerticalError, "m"),
                                      Utils.formatBigValue(LateralSpeed, "m/s")),
                              Styles.boxed_label, GUILayout.ExpandWidth(true));
                GUILayout.EndVertical();
            }
        }
    }
}
