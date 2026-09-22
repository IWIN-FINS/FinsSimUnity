using System.Collections.Generic;

namespace FinsSim.Networking
{
    /// <summary>
    /// Optional normalized-throttle target for <see cref="VehicleRosBridge"/>.
    /// Implementations live in integration packages so the ROS bridge itself does
    /// not take a compile-time dependency on a particular vehicle plugin.
    /// </summary>
    public interface IVehicleRosThrusterFallback
    {
        bool IsAvailable { get; }
        bool ApplyNormalized(IList<float> values);
        void Zero();
    }
}
