using System;
using System.Collections.Generic;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    [DefaultExecutionOrder(-5)]
    public class DomainRandomizationCoordinator : MonoBehaviour
    {
        public DomainRandomizationProfile profile;
        public DomainRandomizationMode mode = DomainRandomizationMode.Train;
        public int baseSeed = 12345;
        public bool incrementSeedPerEpisode = true;
        public bool randomizeOnStart;
        public bool autoFindTargets = true;
        public List<MonoBehaviour> targetBehaviours = new List<MonoBehaviour>();

        [Header("Command Line Overrides")]
        public bool applyCommandLineOverrides = true;
        public string seedCommandLineArg = "-fins-dr-seed";
        public string modeCommandLineArg = "-fins-dr-mode";

        [Header("Runtime Debug")]
        [SerializeField] int episodeIndex;
        [SerializeField] int lastSeed;

        readonly List<IEpisodeRandomizable> _targets = new List<IEpisodeRandomizable>();

        public int EpisodeIndex => episodeIndex;
        public int LastSeed => lastSeed;

        void Awake()
        {
            ApplyCommandLineOverrides();
            ResolveTargets();
        }

        void Start()
        {
            if (randomizeOnStart)
            {
                RandomizeForEpisode();
            }
        }

        public void RandomizeForEpisode()
        {
            if (mode == DomainRandomizationMode.Disabled || profile == null)
            {
                return;
            }

            ResolveTargets();
            lastSeed = incrementSeedPerEpisode ? baseSeed + episodeIndex : baseSeed;
            var context = new RandomizationContext(profile, mode, lastSeed, episodeIndex);

            for (int i = 0; i < _targets.Count; i++)
            {
                _targets[i]?.RandomizeForEpisode(context);
            }

            episodeIndex++;
        }

        public void ResolveTargets()
        {
            _targets.Clear();
            for (int i = 0; i < targetBehaviours.Count; i++)
            {
                if (targetBehaviours[i] is IEpisodeRandomizable target && !_targets.Contains(target))
                {
                    _targets.Add(target);
                }
            }

            if (!autoFindTargets)
            {
                return;
            }

            foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour is IEpisodeRandomizable target && !_targets.Contains(target))
                {
                    _targets.Add(target);
                }
            }
        }

        void ApplyCommandLineOverrides()
        {
            if (!applyCommandLineOverrides)
            {
                return;
            }

            string[] args = Environment.GetCommandLineArgs();
            if (TryGetCommandLineValue(args, seedCommandLineArg, out string seedText)
                && int.TryParse(seedText, out int parsedSeed))
            {
                baseSeed = parsedSeed;
            }

            if (TryGetCommandLineValue(args, modeCommandLineArg, out string modeText)
                && Enum.TryParse(modeText, true, out DomainRandomizationMode parsedMode))
            {
                mode = parsedMode;
            }
        }

        static bool TryGetCommandLineValue(string[] args, string key, out string value)
        {
            value = null;
            if (string.IsNullOrEmpty(key))
            {
                return false;
            }

            string keyWithEquals = key + "=";
            for (int i = 0; i < args.Length; i++)
            {
                string arg = args[i];
                if (string.Equals(arg, key, StringComparison.OrdinalIgnoreCase))
                {
                    if (i + 1 < args.Length)
                    {
                        value = args[i + 1];
                        return true;
                    }

                    return false;
                }

                if (arg.StartsWith(keyWithEquals, StringComparison.OrdinalIgnoreCase))
                {
                    value = arg.Substring(keyWithEquals.Length);
                    return true;
                }
            }

            return false;
        }
    }
}
