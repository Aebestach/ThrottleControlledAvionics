//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//

using System;
using System.Diagnostics.CodeAnalysis;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [SuppressMessage("ReSharper", "FieldCanBeMadeReadOnly.Global"),
     SuppressMessage("ReSharper", "MemberCanBePrivate.Global"),
     SuppressMessage("ReSharper", "MemberCanBeProtected.Global"),
     SuppressMessage("ReSharper", "ConvertToConstant.Global")]
    public class ToOrbitExecutor : OrbitalComponent
    {
        [SuppressMessage("ReSharper", "FieldCanBeMadeReadOnly.Global"),
         SuppressMessage("ReSharper", "MemberCanBePrivate.Global"),
         SuppressMessage("ReSharper", "ConvertToConstant.Global")]
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float MaxG = 3;
            [Persistent] public float MinThrottle = 10;
            [Persistent] public float MinClimbTime = 5;
            [Persistent] public float MaxDynPressure = 10f;
            [Persistent] public float AtmDensityOffset = 10f;
            [Persistent] public float AtmDensityCutoff = 0.1f;
            [Persistent] public float AscentEccentricity = 0.3f;
            [Persistent] public float GravityTurnAngle = 30;
            [Persistent] public float GTurnOffset = 0.1f;
            [Persistent] public float MinThrustMod = 0.8f;
            [Persistent] public float MaxThrustMod = 0.99f;
            [Persistent] public float FirstApA = 10f;
            [Persistent] public float CoastRecoverTime = 10f;

            [Persistent] public PIDf_Controller3 PitchPID = new PIDf_Controller3();
            [Persistent] public PIDf_Controller3 ThrottlePID = new PIDf_Controller3();
            [Persistent] public PIDf_Controller3 ApAPID = new PIDf_Controller3();
            [Persistent] public PIDf_Controller3 NormCorrectionPID = new PIDf_Controller3();

            public float AtmDensityInterval;
            public float ThrustModInterval;

            public override void Load(ConfigNode node)
            {
                base.Load(node);
                AtmDensityInterval = AtmDensityOffset - AtmDensityCutoff;
                ThrustModInterval = MaxThrustMod - C.MinThrustMod;
            }
        }

        public static Config C => Config.INST;

        // ReSharper disable UnassignedField.Global
        protected ThrottleControl THR;
        protected AttitudeControl ATC;
        protected BearingControl BRC;
        // ReSharper restore UnassignedField.Global

        protected readonly SingleAction GearAction = new SingleAction();
        protected readonly FuzzyThreshold<double> ErrorThreshold = new FuzzyThreshold<double>();

        protected Vector3d target;

        public Vector3d Target
        {
            get => target;
            set
            {
                target = value;
                TargetR = target.magnitude;
            }
        }

        public double TargetR { get; private set; }
        [Persistent] public double LaunchUT = -1;
        [Persistent] public double ApAUT = -1;
        [Persistent] public double GravityTurnStart;
        [Persistent] public bool CorrectOnlyAltitude;
        [Persistent] public string LastBodyName = string.Empty;

        [Persistent] public bool AutoTimeToApA = true;
        [Persistent] public FloatField FirstApA = new FloatField(format: "F1");
        [Persistent] public FloatField TimeToApA = new FloatField(format: "F1", min: 5, max: 300);
        [Persistent] public FloatField MaxG = new FloatField(format: "F1", min: 0.1f, max: 100);
        [Persistent] public FloatField MinThrottle = new FloatField(format: "F1", min: 1, max: 100);
        [Persistent] public FloatField MaxDynP = new FloatField(format: "F1", min: 0.1f, max: 300);
        [Persistent] public FloatField MaxAoA = new FloatField(format: "F1", min: 0.1f, max: 90);
        [Persistent] public FloatField GravityTurnAngle = new FloatField(format: "F1", min: 1, max: 45);

        public double MinApR => VesselOrbit.MinPeR() + 1000;

        public double MaxApR => FirstApA * 1000 + Body.Radius;

        public double TimeToClosestApA =>
            VesselOrbit.ApAhead()
                ? VesselOrbit.timeToAp
                : VesselOrbit.timeToAp - VesselOrbit.period;

        public double dApA { get; protected set; } = -1;
        public double dArc { get; protected set; } = -1;
        protected Vector3d htdir, hvdir, ApV;
        protected PIDf_Controller3 pitch = new PIDf_Controller3();
        protected PIDf_Controller3 norm_correction = new PIDf_Controller3();
        protected PIDf_Controller3 throttle = new PIDf_Controller3();
        protected PIDf_Controller3 dApA_pid = new PIDf_Controller3();
        protected double prevApA;
        private float thrustToKeepApA;
        [Persistent] protected bool CoastingToApoapsis;
        [Persistent] protected double CoastTargetApR = -1;
        private double lastCoastApR = -1;
        private double lastCoastUT = -1;
        private float lastCoastThrottle;
        private readonly LowPassFilterD coastApRRatePerThrottle = new LowPassFilterD();
        protected double CircularizationOffset = -1;
        protected bool ApoapsisReached;
        protected bool CourseOnTarget;
        protected float currentAoA;

        public override void Save(ConfigNode node)
        {
            base.Save(node);
            node.AddValue("Target", ConfigNode.WriteVector(target));
        }

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            var tgt = node.GetValue("Target");
            if(!string.IsNullOrEmpty(tgt))
                Target = ConfigNode.ParseVector3D(tgt);
        }

        public ToOrbitExecutor() : base(null)
        {
            ErrorThreshold.Lower = ToOrbitAutopilot.C.Dtol / 10;
            ErrorThreshold.Upper = ToOrbitAutopilot.C.Dtol;
            pitch.setPID(C.PitchPID);
            pitch.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            throttle.setPID(C.ThrottlePID);
            throttle.setClamp(0.5f);
            dApA_pid.setPID(C.ApAPID);
            dApA_pid.setClamp(0.5f);
            norm_correction.setPID(C.NormCorrectionPID);
            norm_correction.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            coastApRRatePerThrottle.Tau = 3;
            FirstApA.Value = -1;
            TimeToApA.Value = TrajectoryCalculator.C.ManeuverOffset;
            MinThrottle.Value = C.MinThrottle;
            GravityTurnAngle.Value = C.GravityTurnAngle;
            MaxG.Value = C.MaxG;
            MaxDynP.Value = C.MaxDynPressure;
            MaxAoA.Value = AttitudeControlBase.C.MaxAttitudeError;
        }

        public virtual void AttachTCA(ModuleTCA tca)
        {
            if(TCA != null)
                return;
            TCA = tca;
            InitModuleFields();
            GearAction.action = () => VSL.GearOn(false);
            TimeToApA.Value = Mathf.Max(TimeToApA, VSL.Torque.MaxCurrent.TurnTime);
            UpdateLimits();
        }

        public void UpdateLimits()
        {
            FirstApA.Min = (float)(MinApR - Body.Radius) / 1000;
            FirstApA.Max = (float)(Body.sphereOfInfluence - Body.Radius) / 1000;
            if(FirstApA < 0 || LastBodyName != Body.bodyName)
                FirstApA.Value = FirstApA.Min + C.FirstApA;
            FirstApA.ClampValue();
            LastBodyName = Body.bodyName;
        }

        public virtual void Reset()
        {
            dApA = dArc = -1;
            LaunchUT = -1;
            ApAUT = -1;
            GravityTurnStart = 0;
            reset_coast();
            ApoapsisReached = false;
            Target = Vector3d.zero;
            ErrorThreshold.Reset();
            pitch.Reset();
            throttle.Reset();
            norm_correction.Reset();
            UpdateLimits();
        }

        protected void update_state(float Dtol)
        {
            hvdir = Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).normalized;
            htdir = Vector3d.Exclude(VesselOrbit.pos, target - VesselOrbit.pos).normalized;
            CourseOnTarget = Vector3d.Dot(htdir, hvdir) > 0.1;
            if(VesselOrbit.ApAhead())
            {
                ApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp);
                dApA = Utils.ClampL(TargetR - VesselOrbit.ApR, 0);
            }
            else
            {
                ApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToPe);
                dApA = Utils.ClampL(TargetR - VesselOrbit.PeR, 0);
            }
            dArc = Utils.ClampL(Utils.ProjectionAngle(ApV, target, htdir) * ToOrbitAutopilot.C.Dtol,
                0);
            ErrorThreshold.Value = CorrectOnlyAltitude ? dApA : dApA + dArc;
            ApoapsisReached |= dApA < Dtol;
        }


        protected void tune_THR()
        {
            THR.CorrectThrottle = ApoapsisReached;
            if(VSL.vessel.dynamicPressurekPa > C.MaxDynPressure)
                THR.MaxThrottle =
                    Mathf.Max(1 - ((float)VSL.vessel.dynamicPressurekPa - C.MaxDynPressure) / 5, 0);
        }

        protected double getStartF()
        {
            if(Body.atmosphere && VSL.vessel.atmDensity > C.AtmDensityOffset)
                GravityTurnStart = VSL.Altitude.Absolute;
            var ApA_f = Utils.Clamp((VesselOrbit.ApA - GravityTurnStart)
                                    / (TargetR - Body.Radius - GravityTurnStart)
                                    / C.GTurnOffset,
                0,
                1);
            return Utils.Clamp(ApA_f
                * (VSL.Engines.WeightedThrustMod - C.MinThrustMod)
                / C.ThrustModInterval, 0, 1);
        }

        protected Vector3d tune_needed_vel(Vector3d needed_vel, Vector3d pg_vel, double startF)
        {
            if(CourseOnTarget && startF > 0)
            {
                var error = (float)(90 - Utils.Angle2(VesselOrbit.GetOrbitNormal(), target));
                norm_correction.Update(error);
                needed_vel = QuaternionD.AngleAxis(norm_correction.Action, VesselOrbit.pos)
                             * needed_vel;
            }
            else
                norm_correction.Update(0);
            var clampAngle = MaxAoA.Value
                             * Utils.ClampL(1 - VSL.vessel.dynamicPressurekPa / MaxDynP.Value, 0);
            return Utils.ClampDirection(needed_vel, pg_vel, clampAngle);
        }

        private void reset_coast()
        {
            CoastingToApoapsis = false;
            CoastTargetApR = -1;
            CircularizationOffset = -1;
            lastCoastApR = -1;
            lastCoastUT = -1;
            lastCoastThrottle = 0;
            coastApRRatePerThrottle.Reset();
        }

        private void start_coast()
        {
            CoastingToApoapsis = true;
            CoastTargetApR = TargetR;
            lastCoastApR = -1;
            lastCoastUT = -1;
            lastCoastThrottle = 0;
            coastApRRatePerThrottle.Reset();
        }

        protected bool ShouldCoast => CoastingToApoapsis || ErrorThreshold;

        private void update_circularization_offset()
        {
            if(CircularizationOffset >= 0)
                return;
            ApAUT = VSL.Physics.UT + VesselOrbit.timeToAp;
            CircularizationOffset = VSL.Engines.TTB_Precise((float)TrajectoryCalculator
                                        .dV4C(VesselOrbit, hV(ApAUT), ApAUT)
                                        .magnitude)
                                    / 2;
        }

        private float max_ascent_throttle()
        {
            var maxThrottle = max_G_throttle();
            if(VSL.vessel.dynamicPressurekPa > C.MaxDynPressure)
                maxThrottle = Mathf.Min(maxThrottle,
                    Mathf.Max(1 - ((float)VSL.vessel.dynamicPressurekPa - C.MaxDynPressure) / 5, 0));
            return Utils.Clamp(maxThrottle, 0, 1);
        }

        private float apoapsis_maintenance_throttle(float Dtol)
        {
            var now = VSL.Physics.UT;
            if(lastCoastUT >= 0 && now > lastCoastUT && lastCoastThrottle > 0.01f)
            {
                var rate = (VesselOrbit.ApR - lastCoastApR) / (now - lastCoastUT) / lastCoastThrottle;
                if(!double.IsNaN(rate) && !double.IsInfinity(rate) && rate > 1)
                    coastApRRatePerThrottle.Update(rate);
            }
            var targetApR = CoastTargetApR > 0 ? CoastTargetApR : TargetR;
            var error = targetApR - VesselOrbit.ApR;
            var tolerance = Math.Max(Dtol, ErrorThreshold.Upper);
            var maxThrottle = max_ascent_throttle();
            var throttle = 0f;
            if(error > tolerance && maxThrottle > 0)
            {
                var targetRecoveryTime = Math.Max(C.CoastRecoverTime, TimeWarp.fixedDeltaTime);
                var desiredRate = error / targetRecoveryTime;
                if(coastApRRatePerThrottle.Value > 1)
                    throttle = (float)(desiredRate / coastApRRatePerThrottle.Value);
                else
                    throttle = MinThrottle / 100;
                var minThrottle = Mathf.Min(MinThrottle / 100, maxThrottle);
                throttle = Utils.Clamp(throttle, minThrottle, maxThrottle);
            }
            lastCoastApR = VesselOrbit.ApR;
            lastCoastUT = now;
            lastCoastThrottle = throttle;
            return throttle;
        }

        protected bool coast(Vector3d pg_vel, float Dtol)
        {
            if(!CoastingToApoapsis)
                start_coast();
            Status("Coasting...");
            CFG.BR.OffIfOn(BearingMode.Auto);
            CFG.AT.OnIfNot(Attitude.Custom);
            ATC.SetThrustDirW(-pg_vel.xzy);
            THR.CorrectThrottle = false;
            update_circularization_offset();
            var keepCoasting = VesselOrbit.timeToAp > TimeToApA + CircularizationOffset
                               && Body.atmosphere
                               && VesselOrbit.radius < Body.Radius + Body.atmosphereDepth;
            THR.Throttle = keepCoasting ? apoapsis_maintenance_throttle(Dtol) : 0;
            prevApA = VesselOrbit.ApA;
            return keepCoasting;
        }

        protected float max_G_throttle()
        {
            var MaxThrust = Vector3.Dot(VSL.Physics.Up, -VSL.Engines.MaxThrust);
            if(MaxThrust <= 0)
                return 1;
            return MaxG
                   / Utils.ClampL((MaxThrust / VSL.Physics.M - VSL.Physics.G) / VSL.Physics.StG,
                       MaxG);
        }

        protected Vector3d get_pg_vel() =>
            Vector3d.Lerp(VesselOrbit.vel,
                VSL.vessel.srf_velocity.xzy,
                VSL.Physics.G / VSL.Physics.StG);

        private Vector3d getAfterApAThrustVector()
        {
            // vis-viva equation to calculate needed orbital velocity
            var PeR = VesselOrbit.radius; // desired PeR
            var vel2 = Body.gravParameter * (2 / PeR - 2 / (TargetR + PeR));
            // desired orbit
            var horizontalDir = Vector3d.Cross(VesselOrbit.GetOrbitNormal(), VesselOrbit.pos).normalized;
            var neededVel = horizontalDir * Math.Sqrt(vel2);
            return neededVel - VesselOrbit.vel;
        }

        protected void auto_ApA_offset()
        {
            if(!AutoTimeToApA)
                return;
            var dEcc = C.AscentEccentricity - (float)VesselOrbit.eccentricity;
            var rel_dApA = (float)((TargetR - VesselOrbit.ApR) / (TargetR - Body.Radius) - 0.1);
            TimeToApA.Value = Mathf.Max(
                TrajectoryCalculator.C.ManeuverOffset * (1 + dEcc + rel_dApA),
                VSL.Torque.MaxCurrent.TurnTime);
            TimeToApA.ClampValue();
        }

        public void UpdateTargetPosition()
        {
            Target = QuaternionD.AngleAxis(Body.angularV * TimeWarp.fixedDeltaTime * Mathf.Rad2Deg,
                         Body.zUpAngularVelocity.normalized)
                     * Target;
        }

        public virtual bool Liftoff()
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateEngines();
            if(VSL.VerticalSpeed.Absolute / VSL.Physics.G < Config.INST.MinClimbTime)
            {
                Status("Liftoff...");
                CFG.DisableVSC();
                CFG.VTOLAssistON = true;
                THR.Throttle = max_G_throttle();
                VSL.OnPlanetParams.ActivateLaunchClamps();
                CFG.AT.OnIfNot(Attitude.Custom);
                var vel = VSL.Physics.Up;
                vel = tune_needed_vel(vel, vel, 1);
                ATC.SetThrustDirW(vel);
                return true;
            }
            StartGravityTurn();
            return false;
        }

        public virtual void StartGravityTurn()
        {
            GravityTurnStart = VSL.Altitude.Absolute;
            ApoapsisReached = false;
            GearAction.Run();
            CFG.VTOLAssistON = false;
            CFG.StabilizeFlight = false;
            CFG.HF.Off();
            prevApA = VesselOrbit.ApA;
            reset_coast();
            update_state(0);
        }

        public bool GravityTurn(float Dtol)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state(Dtol);
            var pg_vel = get_pg_vel();
            currentAoA = Utils.Angle2(VSL.Engines.CurrentDefThrustDir, -(Vector3)pg_vel.xzy);
            // Once coasting starts, keep it latched and correct ApA with small throttle trims.
            if(ShouldCoast)
                return coast(pg_vel, Dtol);
            // the gravity turn proper
            CFG.AT.OnIfNot(Attitude.Custom);
            tune_THR();
            auto_ApA_offset();
            var vel = pg_vel;
            if(VesselOrbit.ApAhead())
            {
                var startF = getStartF();
                var angleOfAscent = Utils.ProjectionAngle(VesselOrbit.pos, pg_vel, target - VesselOrbit.pos);
                var maxAngleOfAscent = VSL.Engines.MaxThrustM > VSL.Physics.mg
                    ? Mathf.Min(Mathf.Acos(VSL.Physics.mg / VSL.Engines.MaxThrustM) * Mathf.Rad2Deg, 45)
                    : 0;
                var neededAngleOfAscent = (float)angleOfAscent;
                if(angleOfAscent < GravityTurnAngle)
                {
                    neededAngleOfAscent = Mathf.Min(
                        maxAngleOfAscent,
                        Mathf.Lerp(
                            GravityTurnAngle,
                            0,
                            (float)(VSL.vessel.atmDensity - C.AtmDensityCutoff) / C.AtmDensityInterval)
                        * (float)startF);
                    pitch.Update((float)angleOfAscent - neededAngleOfAscent);
                    vel = QuaternionD.AngleAxis(
                              pitch,
                              Vector3d.Cross(target, VesselOrbit.pos))
                          * pg_vel;
                }
                vel = tune_needed_vel(vel, pg_vel, startF);
                throttle.Update(TimeToApA - (float)VesselOrbit.timeToAp);
                THR.Throttle = Utils.Clamp(0.5f + throttle, MinThrottle / 100, max_G_throttle());
                if(VSL.vessel.dynamicPressurekPa > 0)
                {
                    dApA_pid.Update((float)(prevApA - VesselOrbit.ApA)/TimeWarp.fixedDeltaTime);
                    thrustToKeepApA = Utils.Clamp(thrustToKeepApA + dApA_pid.Action, 0, 1);
                    THR.Throttle = Math.Max(THR.Throttle, thrustToKeepApA);
                }
                THR.Throttle *= (float)Utils.ClampH(dApA / Dtol / VSL.Engines.TMR, 1);
                if(CFG.AT.Not(Attitude.KillRotation))
                {
                    if(angleOfAscent < GravityTurnAngle)
                    {
                        CFG.BR.OnIfNot(BearingMode.Auto);
                        BRC.ForwardDirection = htdir.xzy;
                    }
                    else
                        CFG.BR.OffIfOn(BearingMode.Auto);
                }
#if DEBUG
                DebugWindowController.PostMessage($"ToOrbit Ex: {VSL.vessel.vesselName}",
                    $"AoA:     {angleOfAscent:F3}\n"
                    + $"needed: {neededAngleOfAscent:F3}\n"
                    + $"max:    {maxAngleOfAscent:F3}\n"
                    + $"Thrust: {VSL.Engines.MaxThrustM:F3}, mg {VSL.Physics.mg:F3}\n"
                    + $"Atm.D:  {VSL.vessel.atmDensity:F3} / {C.AtmDensityCutoff:F3}\n"
                    + $"pitch:  {pitch.LastError:F3} => {pitch.Action:F3}\n"
                    + $"startF: {startF:F3}\n"
                    + $"thrustMod: {((VSL.Engines.WeightedThrustMod - C.MinThrustMod) / C.ThrustModInterval):F3}\n"
                    + $"thr2pg: {currentAoA:F3}\n"
                    + $"need2pg: {Utils.Angle2(vel, pg_vel):F3}\n"
                    + $"ApA thrust: {thrustToKeepApA:P3}\n"
                    + $"<b>ApA pid</b>\n{dApA_pid}");
#endif
            }
            else
            {
                THR.Throttle = 1;
                CFG.BR.OffIfOn(BearingMode.Auto);
                vel = getAfterApAThrustVector();
            }
            ATC.SetThrustDirW(-vel.xzy);
            prevApA = VesselOrbit.ApA;
            Status(Loc.T("ToOrbit_GravityTurn", "Gravity turn..."));
            return true;
        }

        public void DrawOptions()
        {
            draw_float_option(Loc.Content("ToOrbit_MaxApoapsis", "Max. Apoapsis:",
                    "ToOrbit_MaxApoapsis_Tooltip",
                    "The maximum altitude of the starting sub-orbital trajectory "
                    + "that is used to either circularize or to get to a higher orbit."),
                FirstApA, "km", 5, "F1");
            draw_time_to_apoapsis_option();
            draw_float_option(Loc.Content("ToOrbit_MinThrottle", "Min. Throttle:",
                    "ToOrbit_MinThrottle_Tooltip",
                    "Minimum throttle value. Increasing it will shorten the last stage of the ascent."),
                MinThrottle, "%", 5, "F1");
            draw_float_option(Loc.Content("ToOrbit_GTurnAngle", "G.Turn Angle:",
                    "ToOrbit_GTurnAngle_Tooltip",
                    "The initial deviation from vertical direction. After that, the ship will follow prograde. Smaller angle gives steeper trajectory."),
                GravityTurnAngle, "°", 5, "F1");
            draw_float_option(Loc.Content("ToOrbit_MaxAcceleration", "Max. Acceleration:",
                    "ToOrbit_MaxAcceleration_Tooltip",
                    "Maximum allowed acceleration (in gees of the current planet). Smooths gravity turn on low-gravity worlds. Saves fuel."),
                MaxG, "g", 0.5f, "F1");
            draw_float_option(Loc.Content("ToOrbit_MaxDynPressure", "Max. Dyn.Pressure:",
                    "ToOrbit_MaxDynPressure_Tooltip",
                    "Maximum allowed dynamic pressure (for gravity turn in atmosphere). Determines how much the ship is allowed to deviate from prograde. If current dynamic pressure is higher, the ship will follow prograde exactly."),
                MaxDynP, "kPa", 5, "F1");
            draw_float_option(Loc.Content("ToOrbit_MaxAoA", "Max. Angle of Attack:",
                    "ToOrbit_MaxAoA_Tooltip",
                    "Maximum allowed angle of attack. This is the hard limit that is modified by maximum dynamic pressure setting."),
                MaxAoA, "°", 1, "F1");
        }

        static void draw_float_option(GUIContent label, FloatField field, string suffix, float step, string format)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(150));
            field.Draw(suffix, step, format, suffix_width: 25);
            GUILayout.EndHorizontal();
        }

        void draw_time_to_apoapsis_option()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.Content("ToOrbit_TimeToApoapsis", "Time to Apoapsis:",
                    "ToOrbit_TimeToApoapsis_Tooltip",
                    "More time to apoapsis means steeper trajectory and greater acceleration. Low values can save a lot of fuel."),
                GUILayout.Width(150));
            Utils.ButtonSwitch(Loc.T("ToOrbit_Auto", "Auto"),
                ref AutoTimeToApA,
                Loc.T("ToOrbit_Auto_Tooltip", "Tune time to apoapsis automatically"),
                GUILayout.Width(50));
            TimeToApA.Draw("s", 5, "F1", suffix_width: 25);
            GUILayout.EndHorizontal();
        }

        public void DrawInfo(double target_inclination = -1)
        {
            GUILayout.BeginHorizontal();
            {
                GUILayout.BeginVertical();
                {
                    GUILayout.Label(Loc.T("ToOrbit_Inclination", "Inclination:"));
                    GUILayout.Label(Loc.T("ToOrbit_ApoapsisLabel", "Apoapsis:"));
                    GUILayout.Label(Loc.T("ToOrbit_TimeToApoapsisLabel", "Time to Apoapsis:"));
                    GUILayout.Label(Loc.T("ToOrbit_AngleOfAttack", "Angle of Attack:"));
                    GUILayout.Label(Loc.T("ToOrbit_DynPressure", "Dyn. Pressure:"));
                    GUILayout.Label(Loc.T("ToOrbit_EngEfficiency", "Eng. Efficiency:"));
                }
                GUILayout.EndVertical();
                GUILayout.BeginVertical();
                {
                    if(target_inclination >= 0)
                        GUILayout.Label(string.Format("{0:F3}° ► {1:F3}° Err: {2:F3}°",
                            VesselOrbit.inclination,
                            target_inclination,
                            norm_correction.LastError));
                    else
                        GUILayout.Label(string.Format("{0:F3}° Err: {1:F3}°",
                            VesselOrbit.inclination,
                            norm_correction.LastError));
                    GUILayout.Label(string.Format("{0} ► {1}",
                        Utils.formatBigValue((float)VesselOrbit.ApA, "m", "F3"),
                        Utils.formatBigValue((float)(TargetR - Body.Radius), "m", "F3")));
                    GUILayout.Label(string.Format("{0} ► {1}",
                        KSPUtil.PrintDateDeltaCompact(TimeToClosestApA, true, true),
                        KSPUtil.PrintDateDeltaCompact(TimeToApA, true, true)));
                    GUILayout.Label($"{currentAoA:F3}°");
                    GUILayout.Label($"{VSL.vessel.dynamicPressurekPa:F3} kPa");
                    GUILayout.Label($"{VSL.Engines.WeightedThrustMod:P3}");
                }
                GUILayout.EndVertical();
            }
            GUILayout.EndHorizontal();
        }
    }
}
