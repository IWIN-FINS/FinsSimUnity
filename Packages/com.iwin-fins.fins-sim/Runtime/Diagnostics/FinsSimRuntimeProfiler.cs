using System;
using System.Collections.Generic;
using System.Diagnostics;

public static class FinsSimRuntimeProfiler
{
    public struct MarkerStats
    {
        public string Name;
        public int Count;
        public double TotalMs;
        public double MaxMs;
    }

    class Accumulator
    {
        public int Count;
        public long TotalTicks;
        public long MaxTicks;
    }

    static readonly object SyncRoot = new object();
    static readonly Dictionary<string, Accumulator> FrameStats = new Dictionary<string, Accumulator>(StringComparer.Ordinal);
    static readonly Dictionary<string, Accumulator> TotalStats = new Dictionary<string, Accumulator>(StringComparer.Ordinal);

    public static long Begin()
    {
        return Stopwatch.GetTimestamp();
    }

    public static void End(string markerName, long startTicks)
    {
        long elapsedTicks = Stopwatch.GetTimestamp() - startTicks;
        if (elapsedTicks <= 0 || string.IsNullOrEmpty(markerName))
        {
            return;
        }

        lock (SyncRoot)
        {
            Add(FrameStats, markerName, elapsedTicks);
            Add(TotalStats, markerName, elapsedTicks);
        }
    }

    public static List<MarkerStats> SnapshotAndResetFrame()
    {
        lock (SyncRoot)
        {
            List<MarkerStats> snapshot = Snapshot(FrameStats);
            FrameStats.Clear();
            return snapshot;
        }
    }

    public static List<MarkerStats> SnapshotTotal()
    {
        lock (SyncRoot)
        {
            return Snapshot(TotalStats);
        }
    }

    public static void Reset()
    {
        lock (SyncRoot)
        {
            FrameStats.Clear();
            TotalStats.Clear();
        }
    }

    static void Add(Dictionary<string, Accumulator> stats, string markerName, long elapsedTicks)
    {
        if (!stats.TryGetValue(markerName, out Accumulator accumulator))
        {
            accumulator = new Accumulator();
            stats.Add(markerName, accumulator);
        }

        accumulator.Count++;
        accumulator.TotalTicks += elapsedTicks;
        if (elapsedTicks > accumulator.MaxTicks)
        {
            accumulator.MaxTicks = elapsedTicks;
        }
    }

    static List<MarkerStats> Snapshot(Dictionary<string, Accumulator> stats)
    {
        var snapshot = new List<MarkerStats>(stats.Count);
        foreach (KeyValuePair<string, Accumulator> item in stats)
        {
            snapshot.Add(new MarkerStats
            {
                Name = item.Key,
                Count = item.Value.Count,
                TotalMs = TicksToMs(item.Value.TotalTicks),
                MaxMs = TicksToMs(item.Value.MaxTicks),
            });
        }

        return snapshot;
    }

    static double TicksToMs(long ticks)
    {
        return ticks * 1000.0 / Stopwatch.Frequency;
    }
}
