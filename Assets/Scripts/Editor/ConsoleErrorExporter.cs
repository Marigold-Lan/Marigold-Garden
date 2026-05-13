#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace MarigoldGarden.Editor
{
    [InitializeOnLoad]
    public static class ConsoleErrorExporter
    {
        private static readonly List<ConsoleEntry> _entries = new List<ConsoleEntry>();
        private static bool _isCapturing = true;
        private static readonly string OutputDir = Path.Combine(Application.dataPath, "..", "docs", "Console");

        static ConsoleErrorExporter()
        {
            Application.logMessageReceived += OnLogMessageReceived;
        }

        private static void OnLogMessageReceived(string condition, string stackTrace, LogType type)
        {
            if (!_isCapturing) return;
            if (type == LogType.Error || type == LogType.Assert || type == LogType.Exception || type == LogType.Warning)
            {
                _entries.Add(new ConsoleEntry
                {
                    time = DateTime.Now,
                    type = type,
                    message = condition,
                    stackTrace = stackTrace
                });
            }
        }

        [MenuItem("Tools/Export Console Errors #&e", false, 100)]
        public static void ExportErrors()
        {
            // Also grab any entries Unity already collected via reflection (Editor-only logs)
            CollectExistingConsoleEntries();

            if (_entries.Count == 0)
            {
                EditorUtility.DisplayDialog("Export Console Errors",
                    "No errors, warnings, or exceptions captured.\n\nPlay the game or trigger errors first, then use this tool.", "OK");
            }

            Directory.CreateDirectory(OutputDir);

            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string fileName = $"ConsoleReport_{timestamp}.md";
            string filePath = Path.Combine(OutputDir, fileName);

            string content = BuildReport();

            File.WriteAllText(filePath, content, Encoding.UTF8);

            // Also overwrite the "latest" report for convenience
            string latestPath = Path.Combine(OutputDir, "Latest.md");
            File.WriteAllText(latestPath, content, Encoding.UTF8);

            Debug.Log($"[ConsoleErrorExporter] Report exported to docs/Console/{fileName}");

            EditorUtility.RevealInFinder(filePath);
        }

        [MenuItem("Tools/Clear Captured Console Entries")]
        public static void ClearCapturedEntries()
        {
            _entries.Clear();
            Debug.Log("[ConsoleErrorExporter] Captured entries cleared.");
        }

        [MenuItem("Tools/Toggle Console Capture")]
        public static void ToggleCapture()
        {
            _isCapturing = !_isCapturing;
            Debug.Log($"[ConsoleErrorExporter] Capture is now {(_isCapturing ? "ON" : "OFF")}.");
        }

        private static void CollectExistingConsoleEntries()
        {
            // Access Unity's internal ConsoleWindow log entries via reflection
            try
            {
                var logEntriesType = Type.GetType("UnityEditor.LogEntries, UnityEditor");
                if (logEntriesType == null) return;

                var getCountMethod = logEntriesType.GetMethod("GetCount");
                if (getCountMethod == null) return;

                int count = (int)getCountMethod.Invoke(null, null);
                if (count == 0) return;

                var getEntryMethod = logEntriesType.GetMethod("GetEntryInternal");
                var startMethod = logEntriesType.GetMethod("StartGettingEntries");
                var endMethod = logEntriesType.GetMethod("EndGettingEntries");

                if (getEntryMethod == null || startMethod == null || endMethod == null) return;

                startMethod.Invoke(null, null);

                for (int i = 0; i < count; i++)
                {
                    var logEntry = Activator.CreateInstance(Type.GetType("UnityEditor.LogEntry, UnityEditor"));

                    getEntryMethod.Invoke(null, new object[] { i, logEntry });

                    var messageProp = logEntry.GetType().GetProperty("message");
                    var stackTraceProp = logEntry.GetType().GetProperty("stackTrace");
                    var modeProp = logEntry.GetType().GetProperty("mode");

                    if (messageProp == null || stackTraceProp == null || modeProp == null) continue;

                    string msg = messageProp.GetValue(logEntry) as string ?? "";
                    string st = stackTraceProp.GetValue(logEntry) as string ?? "";
                    int mode = (int)modeProp.GetValue(logEntry);

                    // Mode flags: 1=Error, 2=Assert, 4=Log, 8=Warning, 16=Exception
                    // Only capture non-log entries (Error=1, Assert=2, Warning=8, Exception=16)
                    bool isError = (mode & 1) != 0;
                    bool isAssert = (mode & 2) != 0;
                    bool isWarning = (mode & 8) != 0;
                    bool isException = (mode & 16) != 0;

                    if (!isError && !isAssert && !isWarning && !isException) continue;

                    LogType logType = isException ? LogType.Exception
                        : isError ? LogType.Error
                        : isAssert ? LogType.Assert
                        : LogType.Warning;

                    _entries.Add(new ConsoleEntry
                    {
                        time = DateTime.Now,
                        type = logType,
                        message = msg,
                        stackTrace = st
                    });
                }

                endMethod.Invoke(null, null);
            }
            catch
            {
                // Reflection failed silently - runtime captured entries still available
            }
        }

        private static string BuildReport()
        {
            var sb = new StringBuilder();

            int errorCount = 0, warningCount = 0, exceptionCount = 0, assertCount = 0;
            foreach (var entry in _entries)
            {
                switch (entry.type)
                {
                    case LogType.Error: errorCount++; break;
                    case LogType.Warning: warningCount++; break;
                    case LogType.Exception: exceptionCount++; break;
                    case LogType.Assert: assertCount++; break;
                }
            }

            sb.AppendLine($"# Console Error Report");
            sb.AppendLine();
            sb.AppendLine($"**Project:** Marigold Garden");
            sb.AppendLine($"**Generated:** {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            sb.AppendLine($"**Unity Version:** {Application.unityVersion}");
            sb.AppendLine($"**Platform:** {Application.platform}");
            sb.AppendLine();
            sb.AppendLine("## Summary");
            sb.AppendLine();
            sb.AppendLine("| Type | Count |");
            sb.AppendLine("|------|-------|");
            sb.AppendLine($"| Error | {errorCount} |");
            sb.AppendLine($"| Warning | {warningCount} |");
            sb.AppendLine($"| Exception | {exceptionCount} |");
            sb.AppendLine($"| Assert | {assertCount} |");
            sb.AppendLine($"| **Total** | **{_entries.Count}** |");
            sb.AppendLine();

            // Group by error message for easier scanning
            var grouped = new Dictionary<string, List<ConsoleEntry>>();
            foreach (var entry in _entries)
            {
                string key = entry.message.Trim();
                if (string.IsNullOrEmpty(key)) key = "(empty message)";
                if (!grouped.ContainsKey(key))
                    grouped[key] = new List<ConsoleEntry>();
                grouped[key].Add(entry);
            }

            sb.AppendLine("## Grouped Issues");
            sb.AppendLine();

            int groupIndex = 1;
            foreach (var kvp in grouped)
            {
                string typeIcon = GetTypeIcon(kvp.Value[0].type);
                sb.AppendLine($"### {groupIndex}. {typeIcon} {Truncate(kvp.Key, 100)}");
                sb.AppendLine();
                sb.AppendLine($"**Type:** {kvp.Value[0].type} | **Occurrences:** {kvp.Value.Count}");
                sb.AppendLine();

                sb.AppendLine("<details>");
                sb.AppendLine("<summary>Message</summary>");
                sb.AppendLine();
                sb.AppendLine("```");
                sb.AppendLine(kvp.Key);
                sb.AppendLine("```");
                sb.AppendLine("</details>");
                sb.AppendLine();

                sb.AppendLine("<details>");
                sb.AppendLine("<summary>Stack Trace</summary>");
                sb.AppendLine();
                sb.AppendLine("```");
                string stackTrace = kvp.Value[0].stackTrace;
                if (!string.IsNullOrEmpty(stackTrace))
                    sb.AppendLine(stackTrace);
                else
                    sb.AppendLine("(no stack trace)");
                sb.AppendLine("```");
                sb.AppendLine("</details>");
                sb.AppendLine();

                groupIndex++;
            }

            sb.AppendLine("---");
            sb.AppendLine();
            sb.AppendLine("*Generated by ConsoleErrorExporter*");

            return sb.ToString();
        }

        private static string GetTypeIcon(LogType type)
        {
            switch (type)
            {
                case LogType.Error: return "[ERROR]";
                case LogType.Warning: return "[WARN]";
                case LogType.Exception: return "[EXCEPTION]";
                case LogType.Assert: return "[ASSERT]";
                default: return "[LOG]";
            }
        }

        private static string Truncate(string text, int maxLength)
        {
            if (string.IsNullOrEmpty(text)) return text;
            text = text.Replace("\n", " ").Replace("\r", "");
            if (text.Length <= maxLength) return text;
            return text.Substring(0, maxLength) + "...";
        }

        private struct ConsoleEntry
        {
            public DateTime time;
            public LogType type;
            public string message;
            public string stackTrace;
        }
    }
}
#endif
