/*
 * ESP32 High-Precision Motion Control System
 * Task Scheduler
 *
 * Manages real-time tasks with precise timing, allowing for efficient
 * scheduling of control loop, trajectory updates, and auxiliary tasks.
 */

#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <Arduino.h>

#include <functional>
#include <vector>

#include "../Configuration.h"

/**
 * Task function definition
 */
using TaskFunction = std::function<void()>;

/**
 * Task timing mode
 */
enum class TaskTimingMode {
    FIXED_FREQUENCY,  // Task runs at fixed intervals
    ADAPTIVE          // Task timing adapts to system load
};

/**
 * Task information structure
 */
struct TaskInfo {
    TaskFunction function;        // Task function
    uint32_t intervalUs;          // Interval in microseconds
    uint32_t lastExecutionUs;     // Last execution time
    uint32_t executionTimeUs;     // Execution duration
    uint32_t maxExecutionTimeUs;  // Maximum execution time
    uint32_t missedDeadlines;     // Number of missed deadlines
    bool isControlTask;           // Whether this is a high-priority control task
    TaskTimingMode timingMode;    // Task timing mode
    bool enabled;                 // Whether task is enabled
};

/**
 * Task scheduler class for real-time task management
 */
class TaskScheduler {
   public:
    /**
     * Constructor
     */
    TaskScheduler();

    /**
     * Initialize the task scheduler
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Register a control task (high priority, runs on Core 1)
     *
     * @param function Task function
     * @param intervalUs Execution interval in microseconds
     * @param timingMode Task timing mode
     * @return Task index or -1 if failed
     */
    int registerControlTask(TaskFunction function, uint32_t intervalUs,
                            TaskTimingMode timingMode = TaskTimingMode::FIXED_FREQUENCY);

    /**
     * Register an auxiliary task (lower priority, runs on Core 0)
     *
     * @param function Task function
     * @param intervalUs Execution interval in microseconds
     * @param timingMode Task timing mode
     * @return Task index or -1 if failed
     */
    int registerAuxiliaryTask(TaskFunction function, uint32_t intervalUs,
                              TaskTimingMode timingMode = TaskTimingMode::ADAPTIVE);

    /**
     * Execute control tasks that are due
     * This should be called from Core 1 main loop
     *
     * @return Number of tasks executed
     */
    uint32_t executeControlTasks();

    /**
     * Execute auxiliary tasks that are due
     * This should be called from Core 0
     *
     * @return Number of tasks executed
     */
    uint32_t executeAuxiliaryTasks();

    /**
     * Enable a task
     *
     * @param taskIndex Task index
     * @return True if successful, false otherwise
     */
    bool enableTask(int taskIndex);

    /**
     * Disable a task
     *
     * @param taskIndex Task index
     * @return True if successful, false otherwise
     */
    bool disableTask(int taskIndex);

    /**
     * Set task interval
     *
     * @param taskIndex Task index
     * @param intervalUs New interval in microseconds
     * @return True if successful, false otherwise
     */
    bool setTaskInterval(int taskIndex, uint32_t intervalUs);

    /**
     * Get task statistics
     *
     * @param taskIndex Task index
     * @param avgExecutionTimeUs Average execution time
     * @param maxExecutionTimeUs Maximum execution time
     * @param missedDeadlines Number of missed deadlines
     * @return True if successful, false otherwise
     */
    bool getTaskStats(int taskIndex, uint32_t& avgExecutionTimeUs, uint32_t& maxExecutionTimeUs,
                      uint32_t& missedDeadlines);

    /**
     * Reset task statistics
     *
     * @param taskIndex Task index or -1 for all tasks
     * @return True if successful, false otherwise
     */
    bool resetTaskStats(int taskIndex = -1);

    /**
     * Get scheduler statistics
     *
     * @param controlTaskCount Number of control tasks
     * @param auxiliaryTaskCount Number of auxiliary tasks
     * @param totalMissedDeadlines Total missed deadlines
     * @param averageControlLoopTimeUs Average control loop execution time
     */
    void getSchedulerStats(uint32_t& controlTaskCount, uint32_t& auxiliaryTaskCount,
                           uint32_t& totalMissedDeadlines, uint32_t& averageControlLoopTimeUs);

   private:
    // Task lists
    std::vector<TaskInfo> m_controlTasks;
    std::vector<TaskInfo> m_auxiliaryTasks;

    // Statistics
    uint32_t m_totalExecutions;
    uint32_t m_totalExecutionTimeUs;
    uint32_t m_totalMissedDeadlines;
    uint32_t m_lastControlLoopStartUs;
    uint32_t m_controlLoopTimeUs;
    uint32_t m_maxControlLoopTimeUs;

    // Timing
    uint32_t m_lastYieldTimeUs;

    /**
     * Check if a task is due for execution
     *
     * @param task Task information
     * @param currentTimeUs Current time in microseconds
     * @return True if task is due, false otherwise
     */
    bool isTaskDue(const TaskInfo& task, uint32_t currentTimeUs);

    /**
     * Execute a task and update statistics
     *
     * @param task Task information
     * @param currentTimeUs Current time in microseconds
     * @return Execution time in microseconds
     */
    uint32_t executeTask(TaskInfo& task, uint32_t currentTimeUs);
};

#endif  // TASK_SCHEDULER_H