using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PitchPerfect
{
    public class ModulePropController : PartModule
    {
        public static bool usePerfectMode;

        public enum PropControlMode
        {
            Off = 0,
            MaxThrust = 1,
            Proportional = 2,
            MaxEfficiency = 3
            //ConstantRPM = 4
        }

        private bool modeLoaded = false;
        private bool initComplete = false;

        private PropControlMode mode = PropControlMode.MaxThrust;
        public PropControlMode Mode
        {
            get => mode;
            set
            {
                mode = value;
                modeOptionField = (int)mode;
                Fields[nameof(proportionalPercent)].guiActive = Fields[nameof(proportionalPercent)].guiActiveEditor = mode == PropControlMode.Proportional;
                Fields[nameof(feather)].guiActive = Fields[nameof(feather)].guiActiveEditor = mode != PropControlMode.Off;
            }
        }

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Propeller Control")]
        [UI_Toggle(affectSymCounterparts = UI_Scene.All, scene = UI_Scene.All)]
        public bool active = true;
        public bool Active
        {
            get => active;
            set
            {
                active = value;
                Fields[nameof(modeOptionField)].guiActive = active;
                Fields[nameof(proportionalPercent)].guiActive = Fields[nameof(proportionalPercent)].guiActiveEditor = active && Mode == PropControlMode.Proportional;
                Fields[nameof(feather)].guiActive = Fields[nameof(feather)].guiActiveEditor = active;
                Fields[nameof(autofeather)].guiActive = active;
            }
        }

        protected Expansions.Serenity.ModuleRoboticServoRotor rotor;
        protected List<PropControlData> surfaces = new List<PropControlData>();

        public Vector3 RotorAxis => rotor?.transform.TransformVector(rotor.GetMainAxis()).normalized ?? Vector3.zero;

        [UI_FloatRange(affectSymCounterparts = UI_Scene.All, scene = UI_Scene.All, stepIncrement = 0.1f, maxValue = 100, minValue = 0, requireFullControl = true)]
        [KSPAxisField(guiName = "Propeller Thrust", isPersistant = true, guiActiveEditor = true, guiUnits = "%", axisGroup = KSPAxisGroup.MainThrottle, axisMode = KSPAxisMode.Incremental, guiFormat = "0", incrementalSpeed = 100, maxValue = 100, minValue = 0)]
        public float proportionalPercent = 0;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Feather Propellers")]
        [UI_Toggle(affectSymCounterparts = UI_Scene.All, disabledText = "Off", enabledText = "On", scene = UI_Scene.All)]
        public bool feather = false;

        [KSPField(isPersistant = true, advancedTweakable = true, guiActive = true, guiActiveEditor = true, guiName = "Autofeather Propellers")]
        [UI_Toggle(affectSymCounterparts = UI_Scene.All, scene = UI_Scene.All)]
        public bool autofeather = false;

        private bool isAutoFeathering = false;
        private bool wasAlreadyFeathered = false;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "Propeller Target")]
        [UI_ChooseOption(affectSymCounterparts = UI_Scene.All, options = new string[] { "Off", "Thrust", "Throttle", "Efficiency" }, scene = UI_Scene.All, suppressEditorShipModified = true)]
        public int modeOptionField = 1;

        [KSPField(isPersistant = true, advancedTweakable = true, guiActiveEditor = true, guiName = "Compensation for controls")]
        [UI_Toggle(affectSymCounterparts = UI_Scene.All, scene = UI_Scene.Editor, disabledText = "Decrease Lift", enabledText = "Increase Lift")]
        public bool nerfLiftForControls = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Reverse Thrust")]
        [UI_Toggle(affectSymCounterparts = UI_Scene.All, scene = UI_Scene.All, disabledText = "Off", enabledText = "On")]
        public bool reverseThrust = false;

        [KSPAction("Cycle Propeller Control Modes")]
        public void ChangePropMode(KSPActionParam param) => Mode = (PropControlMode)(((int)Mode + 1) % 4);

        [KSPAction("Toggle Propeller Control")]
        public void TogglePropControl(KSPActionParam param) => Active = !Active;

        [KSPAction("Disable Propeller Control")]
        public void DisablePropellerControlAction(KSPActionParam param) => Active = false;

        [KSPAction("Enable Propeller Control")]
        public void EnablePropellerControlAction(KSPActionParam param) => Active = true;

        [KSPAction("Set Propeller Control Mode to Thrust")]
        public void PropControlThrustAction(KSPActionParam param) => Mode = PropControlMode.MaxThrust;

        [KSPAction("Set Propeller Control Mode to Proportional")]
        public void PropControlProportionalAction(KSPActionParam param) => Mode = PropControlMode.Proportional;

        [KSPAction("Set Propeller Control Mode to Efficiency")]
        public void PropControlEfficiencyAction(KSPActionParam param) => Mode = PropControlMode.MaxEfficiency;

        [KSPAction("Feather Propellers")]
        public void FeatherPropsAction(KSPActionParam param) => feather = true;

        [KSPAction("Unfeather Propellers")]
        public void UnfeatherPropsAction(KSPActionParam param) => feather = false;

        [KSPAction("Toggle feather propellers")]
        public void ToggleGeatherPropsAction(KSPActionParam param) => feather = !feather;

        [KSPAction("Reverse Thrust", actionGroup = KSPActionGroup.Brakes)]
        public void ReverseThrust(KSPActionParam param)
        {
            KSPActionType type = param.type;
            if (type == KSPActionType.Activate)
                reverseThrust = true;
            else if (type == KSPActionType.Deactivate)
                reverseThrust = false;
            else
                reverseThrust = !reverseThrust;
        }

        public override void OnStartFinished(StartState state)
        {
            base.OnStartFinished(state);
            rotor = part.FindModuleImplementing<Expansions.Serenity.ModuleRoboticServoRotor>();
            if (rotor == null)
            {
                Debug.LogErrorFormat("ModulePropController must be placed on a part that has a module deriving from ModuleRoboticServoRotor.\t{0}", part.partName);
                Destroy(this);
                return;
            }
            if (HighLogic.LoadedSceneIsFlight)
                InitSurfaces();

            initComplete = true;
        }

        public void SetModeFromIntObject(object value)
        {
            Mode = (PropControlMode)value;
            if (HighLogic.LoadedSceneIsEditor)
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }
        public void SetActiveFromBoolObject(object value)
        {
            Active = (bool)value;
        }

        protected void OnInitialPlacement(Part partPlaced)
        {
            if (partPlaced != this.part)
                return;
            if (modeLoaded)
                return;

            Vector3 axis = RotorAxis;
            // If it's likely to be a propeller, default to max thrust.
            if (Vector3.Angle(axis, EditorLogic.VesselRotation * Vector3.forward) <= 30)
                Mode = PropControlMode.MaxThrust;
            // If it's likely to be a helicopter rotor, default to proportional mode.
            else if (Vector3.Angle(axis, EditorLogic.VesselRotation * Vector3.up) <= 30)
                Mode = PropControlMode.Proportional;
            // Otherwise, disable because someone is doing something very Kerbal
            // and I won't even try to guess at their plans.
            else
                Mode = PropControlMode.Off;

            modeLoaded = true;
        }

        public void FixedUpdate()
        {
            if (!initComplete)
                return;
            if (!HighLogic.LoadedSceneIsFlight || FlightDriver.Pause)
                return;
            if (mode == PropControlMode.Off || !Active)
                return;

            // No sense doing all the math for nothing.
            if (!vessel.mainBody.atmosphere || vessel.altitude > vessel.mainBody.atmosphereDepth)
            {
                // Set blade angles to 0;
                for (int i = surfaces.Count - 1; i >= 0; i--)
                    if (surfaces[i].surface.deploy)
                        surfaces[i].surface.deployAngle = 0;
                return;
            }

#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample("ModulePropController.FixedUpdate()");
#endif

            if (autofeather)
            {
                if (!isAutoFeathering && (!rotor.servoMotorIsEngaged || rotor.rpmLimit == 0 || rotor.maxTorque == 0 || rotor.servoIsLocked))
                {
                    isAutoFeathering = true;
                    wasAlreadyFeathered = feather;
                    feather = true;
                }
                else if (isAutoFeathering && !(!rotor.servoMotorIsEngaged || rotor.rpmLimit == 0 || rotor.maxTorque == 0 || rotor.servoIsLocked))
                {
                    isAutoFeathering = false;
                    if (!wasAlreadyFeathered)
                        feather = false;
                }
            }

            if (false)//mode == PropControlMode.ConstantRPM || mode == PropControlMode.Proportional)
            {
                // Control engine torque to maintain max speed efficiently.
            }
            if (mode == PropControlMode.MaxEfficiency || mode == PropControlMode.MaxThrust || mode == PropControlMode.Proportional)
            {
                if (usePerfectMode)
                    InitSurfaces();
                SetBladeAngles();
            }

#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif
        }

        protected void SetBladeAngles()
        {
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample("ModulePropController.SetBladeAngles()");
#endif
            // For 'realism' we only take the inflow velocity normal to the propeller disc.
            // For propellers, this averages out the target angle around the circle so that
            // the target angle is a function only of forward speed and RPM.
            // For rotors, this shouldn't be an issue since there will be active control.
            Vector3 axis = RotorAxis;
            Vector3 ownPartVelocity = this.part.Rigidbody.GetPointVelocity(rotor.transform.position);
            Vector3 inflowVelocity = Vector3.Project(
                ownPartVelocity + Krakensbane.GetFrameVelocityV3f(), axis);

            foreach (PropControlData prop in surfaces)
            {
                // Remember to skip surfaces that aren't deployed.
                if (!prop.surface.deploy)
                    continue;

                float dot;
                if (feather)
                    dot = prop.zeroLiftDot;
                else
                {
                    switch (mode)
                    {
                        case PropControlMode.MaxEfficiency:
                            dot = prop.maxLDDot;
                            break;
                        case PropControlMode.Proportional:
                            dot = prop.GetProportionalDot(proportionalPercent * 0.01f);
                            break;
                        case PropControlMode.MaxThrust:
                        default:
                            dot = prop.maxLiftDot;
                            break;
                    }
                }

                prop.surface.SetupCoefficients(Vector3.zero, out _, out Vector3 liftVector, out _, out _);
                Vector3 surfaceRefPoint = prop.SurfaceRefPoint;

                //Vector3 velocityVect = (inflowVelocity + Vector3.Cross(rotor.GetMainAxis() * rotor.currentRPM * (2 * Mathf.PI / 60), surfaceRefPoint - rotor.transform.position)).normalized;
                Vector3 velocityVect = (Vector3.ProjectOnPlane(
                        prop.surface.part.Rigidbody.GetPointVelocity(prop.SurfaceRefPoint) - ownPartVelocity,
                        axis) +
                    inflowVelocity);

                if (velocityVect.magnitude < 5)
                {
                    prop.surface.deployAngle = 0;
                    continue;
                }
                velocityVect = velocityVect.normalized;

                Vector3 pitchAxis = prop.surfaceTransform.rotation * Vector3.right;

                if (PropControlMath.GetDeployAngle(velocityVect, pitchAxis, liftVector, dot, out float deployAngle, reverseThrust))
                {
                    // Remember to decrease angle as necessary for surfaces that don't ignore all control inputs.
                    if (!prop.surface.ignorePitch || !prop.surface.ignoreRoll || !prop.surface.ignoreYaw)
                    {
                        // Stall protection:
                        float maxLiftAngle;
                        if (Mode != PropControlMode.MaxThrust || !(Mode == PropControlMode.Proportional && proportionalPercent < 100))
                        {
                            if (!PropControlMath.GetDeployAngle(velocityVect, pitchAxis, liftVector, prop.maxLiftDot, out maxLiftAngle, reverseThrust))
                                maxLiftAngle = deployAngle;
                        }
                        else
                            maxLiftAngle = deployAngle;

                        float controlInput;
                        if (nerfLiftForControls)
                            controlInput = 1;
                        else
                        {
                            controlInput = Math.Max(Math.Max(
                                    prop.surface.ignorePitch ? 0 : Math.Abs(vessel.ctrlState.pitch),
                                    prop.surface.ignoreRoll ? 0 : Math.Abs(vessel.ctrlState.roll)),
                                    prop.surface.ignoreYaw ? 0 : Math.Abs(vessel.ctrlState.yaw));
                        }
                        if (controlInput > 0)
                        {
                            if (!reverseThrust)
                                deployAngle = Math.Min(deployAngle, maxLiftAngle - controlInput * prop.surface.ctrlSurfaceRange * prop.surface.authorityLimiter * 0.01f);
                            else
                                deployAngle = Math.Max(deployAngle, maxLiftAngle - controlInput * prop.surface.ctrlSurfaceRange * prop.surface.authorityLimiter * 0.01f);
                            if (PropControlMath.GetDeployAngle(velocityVect, pitchAxis, liftVector, prop.zeroLiftDot, out float zeroAngle, reverseThrust))
                                deployAngle = !reverseThrust ? Math.Max(deployAngle, zeroAngle) : Math.Min(deployAngle, zeroAngle);
                            else
                                deployAngle = !reverseThrust ? Math.Max(deployAngle, 0) : Math.Min(deployAngle, 0);
                        }
                    }

                    prop.surface.deployAngle = Mathf.Clamp(deployAngle, prop.surface.deployAngleLimits[0], prop.surface.deployAngleLimits[1]);
                }
                //else
                //    Debug.Log("PropControl couldn't set a reasonable angle.");
            }
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif
        }

        protected void InitSurfaces()
        {
#if ENABLE_PROFILER
            System.Diagnostics.Stopwatch timer = new System.Diagnostics.Stopwatch();
            timer.Start();
            UnityEngine.Profiling.Profiler.BeginSample("ModulePropController.InitSurfaces()");
#endif

            surfaces.Clear();

            List<ModuleControlSurface> ctrlSurfaces = part.children
                .Where(p => p.HasModuleImplementing<ModuleControlSurface>())
                .Select(p => p.FindModuleImplementing<ModuleControlSurface>())
                .ToList();

            Vector3 axis = RotorAxis;
            Vector3 rotationOrigin = rotor.transform.position;

            for (int i = 0; i < ctrlSurfaces.Count; i++)//(ModuleControlSurface surface in ctrlSurfaces)
            {
                ModuleControlSurface surface = ctrlSurfaces[i];
                // Get max lift point.
                // Get max L/D point.
                // Get 0 lift point.
                // Get 25 samples for proportional control.
                PropControlData surfaceData = new PropControlData(surface, axis, rotationOrigin);
                surfaces.Add(surfaceData);
                if (surface.part.symMethod == SymmetryMethod.Radial)
                {
                    foreach (ModuleControlSurface symmetrySurface in surface.part.symmetryCounterparts.Select(p => p.FindModuleImplementing<ModuleControlSurface>()))
                    {
                        surfaces.Add(new PropControlData(symmetrySurface, surfaceData));
                        ctrlSurfaces.Remove(symmetrySurface);
                    }
                }
            }
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
            timer.Stop();
            Debug.LogFormat("PropControl: Initialize Time: {0}", timer.Elapsed.TotalMilliseconds);
#endif
        }

        public override void OnAwake()
        {
            base.OnAwake();
            ConfigNode settingsNode = GameDatabase.Instance.GetConfigNodes("PitchPerfectSettings").FirstOrDefault();
            if (settingsNode != null)
            {
                if (!bool.TryParse(settingsNode.GetValue("usePerfectMode"), out usePerfectMode))
                    usePerfectMode = false;
                if (int.TryParse(settingsNode.GetValue("proportionCount"), out int readValue))
                    PropControlData.proportionCount = readValue;
            }
            Debug.LogFormat("PropControl: {0}", usePerfectMode ? "Perfect Mode" : "Close Approximation");

            if (HighLogic.LoadedSceneIsFlight)
            {
                GameEvents.OnEVAConstructionModePartAttached.Add(OnPartAttached);
                GameEvents.OnEVAConstructionModePartDetached.Add(OnPartDetached);
                GameEvents.onPartAttach.Add(OnPartAttached);
                GameEvents.onPartDestroyed.Add(OnPartDestroyed);
                GameEvents.onPartJointBreak.Add(OnPartJointBreak);
            }
            else if (HighLogic.LoadedSceneIsEditor)
            {
                GameEvents.onEditorPartPlaced.Add(OnInitialPlacement);
            }
            Fields[nameof(modeOptionField)].OnValueModified += SetModeFromIntObject;
            Fields[nameof(active)].OnValueModified += SetActiveFromBoolObject;
        }

        public void OnDestroy()
        {
            if (HighLogic.LoadedSceneIsFlight)
            {
                GameEvents.OnEVAConstructionModePartAttached.Remove(OnPartAttached);
                GameEvents.OnEVAConstructionModePartDetached.Remove(OnPartDetached);
                GameEvents.onPartAttach.Remove(OnPartAttached);
                GameEvents.onPartDestroyed.Remove(OnPartDestroyed);
                GameEvents.onPartJointBreak.Remove(OnPartJointBreak);
            }
            else if (HighLogic.LoadedSceneIsEditor)
            {
                GameEvents.onEditorPartPlaced.Remove(OnInitialPlacement);
            }
            Fields[nameof(modeOptionField)].OnValueModified -= SetModeFromIntObject;
            Fields[nameof(active)].OnValueModified -= SetActiveFromBoolObject;
        }

        public void OnPartAttached(Vessel vesselConstructed, Part partAttached)
        {
            if (vesselConstructed != vessel)
                return;
            if (partAttached.parent != part)
                return;
            ModuleControlSurface surface = partAttached.FindModuleImplementing<ModuleControlSurface>();
            if (surface != null)
            {
                surfaces.Add(new PropControlData(surface, RotorAxis, rotor.transform.position));
            }
        }

        public void OnPartAttached(GameEvents.HostTargetAction<Part, Part> data)
        {
            if (data.host != part)
                return;
            ModuleControlSurface surface = data.target.FindModuleImplementing<ModuleControlSurface>();
            if (surface != null)
            {
                surfaces.Add(new PropControlData(surface, RotorAxis, rotor.transform.position));
            }
        }

        public void OnPartDetached(Vessel vesselConstructing, Part partDetached)
        {
            if (vesselConstructing != vessel)
                return;
            surfaces.RemoveAll(p => p.surface.part == partDetached);
        }

        public void OnPartDestroyed(Part partDestroyed)
        {
            surfaces.RemoveAll(p => p.surface.part == partDestroyed);
        }

        public void OnPartJointBreak(PartJoint joint, float value)
        {
            if (joint.Parent == part)
                surfaces.RemoveAll(p => p.surface.part == joint.Child);
        }

        public override void OnSave(ConfigNode node)
        {
            base.OnSave(node);
            node.AddValue("mode", (int)Mode);
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            if (node.HasValue("mode") && int.TryParse(node.GetValue("mode"), out int value))
            {
                if (value >= 5 || value < 0)
                    Mode = PropControlMode.Off;
                else
                    Mode = (PropControlMode)value;
                modeLoaded = true;
            }
        }
    }
}
