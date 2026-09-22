using UnityEngine;
using Conditional = System.Diagnostics.ConditionalAttribute;
using Object = UnityEngine.Object;

public static class ThreeChaseOneRuntimeLog
{
    public static bool ConsoleLogsCompiledIn
    {
        get
        {
#if UNITY_EDITOR || DEVELOPMENT_BUILD || THREE_CHASE_ONE_RUNTIME_LOGS
            return true;
#else
            return false;
#endif
        }
    }

    [Conditional("UNITY_EDITOR")]
    [Conditional("DEVELOPMENT_BUILD")]
    [Conditional("THREE_CHASE_ONE_RUNTIME_LOGS")]
    public static void Info(string message, Object context = null)
    {
        if (context != null)
        {
            UnityEngine.Debug.Log(message, context);
            return;
        }

        UnityEngine.Debug.Log(message);
    }

    [Conditional("UNITY_EDITOR")]
    [Conditional("DEVELOPMENT_BUILD")]
    [Conditional("THREE_CHASE_ONE_RUNTIME_LOGS")]
    public static void Warning(string message, Object context = null)
    {
        if (context != null)
        {
            UnityEngine.Debug.LogWarning(message, context);
            return;
        }

        UnityEngine.Debug.LogWarning(message);
    }
}
