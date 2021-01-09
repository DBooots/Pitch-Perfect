using System;
using Accord.Math.Optimization;
using UnityEngine;

namespace PitchPerfect
{
    public struct PropControlData
    {
        internal static int proportionCount = 10;

        public readonly ModuleControlSurface surface;
        public readonly Transform surfaceTransform;
        public readonly float maxLiftDot;
        public readonly float maxLDDot;
        public readonly float zeroLiftDot;
        public readonly float[] proportionalDots;

        public Vector3 SurfaceRefPoint
        {
            get => surface.displaceVelocity ?
                surface.transform.TransformPoint(surface.velocityOffset) : surface.transform.position;
        }
        public Vector3 PitchAxis
        {
            get => surfaceTransform.rotation * Vector3.right;
        }

        private PropControlData(ModuleControlSurface surface) : this()
        {
            this.surface = surface;
            this.surfaceTransform = surface.transform;
        }

        public PropControlData(ModuleControlSurface surface, PropControlData basis) : this(surface)
        {
            this.maxLiftDot = basis.maxLiftDot;
            this.maxLDDot = basis.maxLDDot;
            this.zeroLiftDot = basis.zeroLiftDot;
            this.proportionalDots = basis.proportionalDots;
        }

        public PropControlData(ModuleControlSurface surface, Vector3 axis, Vector3 rotationOrigin) : this(surface)
        {
            Vector3 pitchAxis = surfaceTransform.rotation * Vector3.right;
            surface.SetupCoefficients(Vector3.zero, out _, out Vector3 liftVector_0, out _, out _);
            Vector3 actionPoint = (surface.displaceVelocity ?
                surface.transform.TransformPoint(surface.velocityOffset) : surface.transform.position) - rotationOrigin;

            double ThrustFunc(double sinAlpha)
            {
                float lift = surface.liftCurve.Evaluate((float)sinAlpha);
                return lift * Vector3.Project(Quaternion.AngleAxis(Mathf.Asin((float)sinAlpha) * Mathf.Rad2Deg, pitchAxis) * liftVector_0, liftVector_0).magnitude;
            }
            double EfficiencyFunc(double sinAlpha)
            {
                float lift = surface.liftCurve.Evaluate((float)sinAlpha);
                float drag = surface.dragCurve.Evaluate((float)sinAlpha);
                Vector3 LiftVector = Quaternion.AngleAxis(Mathf.Asin((float)sinAlpha) * Mathf.Rad2Deg, pitchAxis) * liftVector_0;
                return (lift * Vector3.Project(LiftVector, axis).magnitude) / (lift * Vector3.Project(Vector3.Cross(LiftVector, actionPoint.normalized), axis).magnitude + drag);
            }

            InitializeDots(surface, ThrustFunc, EfficiencyFunc, out maxLiftDot, out maxLDDot, out zeroLiftDot, out proportionalDots);

#if DEBUG
            //Debug.LogFormat("Prop Initialized: {0}\tMax Lift:\t{1}\tZero Lift:\t{2}\tMax L/D:\t{3}", surface.part.partName, maxLiftDot, zeroLiftDot, maxLDDot);
#endif
        }

        private static void InitializeDots(ModuleControlSurface surface, Func<double,double> thrustFunc, Func<double,double> efficiencyFunc, out float maxLiftDot, out float maxLDDot, out float zeroLiftDot, out float[] proportionalDots, float singleProportionalIndex = -1)
        {
            float upperBound = 1, maxKeyValue = 0;
            for (int i = 0; i < surface.liftCurve.Curve.keys.Length; i++)
            {
                float keyValue = surface.liftCurve.Curve.keys[i].value;
                if (keyValue < maxKeyValue)
                {
                    upperBound = surface.liftCurve.Curve.keys[i].time;
                    break;
                }
                maxKeyValue = keyValue;
            }
            BrentSearch ThrustSolver = new BrentSearch(thrustFunc, 0, upperBound);
            proportionalDots = new float[proportionCount + 1];
            bool failedSolver = false;
            if (ThrustSolver.Maximize())
            {
                //Debug.LogFormat("PropControl Maximize iterations: {0}", ThrustSolver.Iterations);
                maxLiftDot = (float)ThrustSolver.Solution;
                ThrustSolver.UpperBound = maxLiftDot;
                float maxLiftValue = (float)ThrustSolver.Value;
                proportionalDots[proportionCount] = maxLiftDot;
                for (int i = 1; i < proportionCount; i++)
                {
                    if (!ThrustSolver.Find(i * maxLiftValue / proportionCount))
                    {
                        //Debug.LogFormat("PropControl Init Failure: {0}\t{1}\t{2}", i, i * maxLiftValue / proportionCount, ThrustSolver.Iterations);
                        failedSolver = true;
                        break;
                    }
                    proportionalDots[i] = (float)ThrustSolver.Solution;
                    //Debug.LogFormat("PropControl Init: {0}\t{1}\t{2}\t{3}", i, i * maxLiftValue / proportionCount, proportionalDots[i], ThrustSolver.Iterations);
                }
                if (ThrustSolver.FindRoot())
                    zeroLiftDot = (float)ThrustSolver.Solution;
                else
                    zeroLiftDot = 0;
                proportionalDots[0] = zeroLiftDot;
            }
            else
            {
                maxLiftDot = (float)ThrustSolver.Solution; // An invalid solution but I'm required to definitevly set this, even though it will be reset later.
                zeroLiftDot = 0; // Same here.
                failedSolver = true;
            }

            if (failedSolver)
            {
                //Debug.LogFormat("PropControl failed to resolve Thrust for {0} [{1}].", surface.part.partName, surface.vessel.parts.IndexOf(surface.part));
                float maxLiftValue = 0;
                maxLiftDot = 0;
                for (int i = 0; i < surface.liftCurve.Curve.length; i++)
                {
                    if (surface.liftCurve.Curve.keys[i].value > maxLiftValue)
                        maxLiftValue = surface.liftCurve.Curve.keys[i].value;
                    else
                        maxLiftDot = surface.liftCurve.Curve.keys[i].time;
                }
                for (int i = 0; i <= proportionCount; i++)
                    proportionalDots[i] = i * maxLiftDot / proportionCount;
                zeroLiftDot = 0;
            }

            BrentSearch EfficiencySolver = new BrentSearch(efficiencyFunc, 0, maxLiftDot);
            if (EfficiencySolver.Maximize())
                maxLDDot = (float)EfficiencySolver.Solution;
            else
            {
                //Debug.LogFormat("PropControl failed to resolve L/D for {0} [{1}].", surface.part.partName, surface.vessel.parts.IndexOf(surface.part));
                maxLDDot = maxLiftDot;
            }
            //Debug.LogFormat("PropControl Efficiency iterations: {0}", EfficiencySolver.Iterations);
        }

        public float GetProportionalDot(float proportion)
        {
            if (proportion >= 1)
                return maxLiftDot;
            if (proportion <= 0)
                return zeroLiftDot;
            int lowerIndex = Mathf.FloorToInt(proportion * (proportionalDots.Length - 1));
            proportion -= (float)lowerIndex / (proportionalDots.Length - 1);
            proportion *= (proportionalDots.Length - 1);
            return Mathf.Lerp(proportionalDots[lowerIndex], proportionalDots[lowerIndex + 1], proportion);
        }
    }
}
