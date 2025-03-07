/*
 * ESP32 High-Precision Motion Control System
 * Task Scheduler Implementation
 */

#include "TaskScheduler.h"

TaskScheduler::TaskScheduler()
    : m_totalExecutions(0),
      m_totalExecutionTimeUs(0),
      m_totalMissedDeadlines(0),
      m_lastControlLoopStartUs(0),
      m_controlLoopTimeUs(0),
      m_maxControlLoopTimeUs(0),
      m_lastYieldTimeUs(0) {
    // Reserve memory for tasks to avoid reallocations
    m_controlTasks.reserve(16);
    m_auxiliaryTasks.reserve(16);
}

bool TaskScheduler::initialize() {
    // Reset timing variables
    m_lastControlLoopStartUs = micros();
    m_lastYieldTimeUs = m_lastControlLoopStartUs;

    return true;
}

int TaskScheduler::registerControlTask(TaskFunction function, uint32_t intervalUs,
                                       TaskTimingMode timingMode) {
    if (!function || intervalUs == 0) {
        return -1;
    }

    // Create new task info
    TaskInfo task;
    task.function = function;
    task.intervalUs = intervalUs;
    task.lastExecutionUs = micros();
    task.executionTimeUs = 0;
    task.maxExecutionTimeUs = 0;
    task.missedDeadlines = 0;
    task.isControlTask = true;
    task.timingMode = timingMode;
    task.enabled = true;

    // Add to control tasks
    m_controlTasks.push_back(task);

    return static_cast<int>(m_controlTasks.size() - 1);
}

int TaskScheduler::registerAuxiliaryTask(TaskFunction function, uint32_t intervalUs,
                                         TaskTimingMode timingMode) {
    if (!function || intervalUs == 0) {
        return -1;
    }

    // Create new task info
    TaskInfo task;
    task.function = function;
    task.intervalUs = intervalUs;
    task.lastExecutionUs = micros();
    task.executionTimeUs = 0;
    task.maxExecutionTimeUs = 0;
    task.missedDeadlines = 0;
    task.isControlTask = false;
    task.timingMode = timingMode;
    task.enabled = true;

    // Add to auxiliary tasks
    m_auxiliaryTasks.push_back(task);

    return static_cast<int>(m_auxiliaryTasks.size() - 1);
}

uint32_t TaskScheduler::executeControlTasks() {
    uint32_t currentTimeUs = micros();
    uint32_t tasksExecuted = 0;

    // Record control loop start time
    m_lastControlLoopStartUs = currentTimeUs;

    // Execute control tasks that are due
    for (auto& task : m_controlTasks) {
        if (task.enabled && isTaskDue(task, currentTimeUs)) {
            // Execute task and measure time
            uint32_t executionTimeUs = executeTask(task, currentTimeUs);

            // Update statistics
            tasksExecuted++;

            // Check if we exceeded interval (missed deadline)
            if (executionTimeUs > task.intervalUs) {
                task.missedDeadlines++;
                m_totalMissedDeadlines++;
            }

            // Update current time
            currentTimeUs = micros();
        }
    }

    // Calculate control loop time
    m_controlLoopTimeUs = micros() - m_lastControlLoopStartUs;

    // Update max control loop time
    if (m_controlLoopTimeUs > m_maxControlLoopTimeUs) {
        m_maxControlLoopTimeUs = m_controlLoopTimeUs;
    }

    return tasksExecuted;
}

uint32_t TaskScheduler::executeAuxiliaryTasks() {
    uint32_t currentTimeUs = micros();
    uint32_t tasksExecuted = 0;

    // Execute auxiliary tasks that are due
    for (auto& task : m_auxiliaryTasks) {
        if (task.enabled && isTaskDue(task, currentTimeUs)) {
            // Execute task and measure time
            uint32_t executionTimeUs = executeTask(task, currentTimeUs);

            // Update statistics
            tasksExecuted++;

            // For auxiliary tasks, we don't count missed deadlines
            // since they are lower priority

            // Update current time
            currentTimeUs = micros();
        }
    }

    return tasksExecuted;
}

bool TaskScheduler::enableTask(int taskIndex) {
    // Check if task index is valid
    if (taskIndex >= 0) {
        // Check in control tasks
        if (static_cast<size_t>(taskIndex) < m_controlTasks.size()) {
            m_controlTasks[taskIndex].enabled = true;
            return true;
        }

        // Check in auxiliary tasks
        if (static_cast<size_t>(taskIndex) < m_auxiliaryTasks.size()) {
            m_auxiliaryTasks[taskIndex].enabled = true;
            return true;
        }
    }

    return false;
}

bool TaskScheduler::disableTask(int taskIndex) {
    // Check if task index is valid
    if (taskIndex >= 0) {
        // Check in control tasks
        if (static_cast<size_t>(taskIndex) < m_controlTasks.size()) {
            m_controlTasks[taskIndex].enabled = false;
            return true;
        }

        // Check in auxiliary tasks
        if (static_cast<size_t>(taskIndex) < m_auxiliaryTasks.size()) {
            m_auxiliaryTasks[taskIndex].enabled = false;
            return true;
        }
    }

    return false;
}

bool TaskScheduler::setTaskInterval(int taskIndex, uint32_t intervalUs) {
    // Validate inputs
    if (taskIndex < 0 || intervalUs == 0) {
        return false;
    }

    // Check in control tasks
    if (static_cast<size_t>(taskIndex) < m_controlTasks.size()) {
        m_controlTasks[taskIndex].intervalUs = intervalUs;
        return true;
    }

    // Check in auxiliary tasks
    if (static_cast<size_t>(taskIndex) < m_auxiliaryTasks.size()) {
        m_auxiliaryTasks[taskIndex].intervalUs = intervalUs;
        return true;
    }

    return false;
}

bool TaskScheduler::getTaskStats(int taskIndex, uint32_t& avgExecutionTimeUs,
                                 uint32_t& maxExecutionTimeUs, uint32_t& missedDeadlines) {
    // Validate inputs
    if (taskIndex < 0) {
        return false;
    }

    // Check in control tasks
    if (static_cast<size_t>(taskIndex) < m_controlTasks.size()) {
        avgExecutionTimeUs = m_controlTasks[taskIndex].executionTimeUs;
        maxExecutionTimeUs = m_controlTasks[taskIndex].maxExecutionTimeUs;
        missedDeadlines = m_controlTasks[taskIndex].missedDeadlines;
        return true;
    }

    // Check in auxiliary tasks
    if (static_cast<size_t>(taskIndex) < m_auxiliaryTasks.size()) {
        avgExecutionTimeUs = m_auxiliaryTasks[taskIndex].executionTimeUs;
        maxExecutionTimeUs = m_auxiliaryTasks[taskIndex].maxExecutionTimeUs;
        missedDeadlines = m_auxiliaryTasks[taskIndex].missedDeadlines;
        return true;
    }

    return false;
}

bool TaskScheduler::resetTaskStats(int taskIndex) {
    // Reset all tasks
    if (taskIndex < 0) {
        for (auto& task : m_controlTasks) {
            task.executionTimeUs = 0;
            task.maxExecutionTimeUs = 0;
            task.missedDeadlines = 0;
        }

        for (auto& task : m_auxiliaryTasks) {
            task.executionTimeUs = 0;
            task.maxExecutionTimeUs = 0;
            task.missedDeadlines = 0;
        }

        m_totalExecutions = 0;
        m_totalExecutionTimeUs = 0;
        m_totalMissedDeadlines = 0;
        m_maxControlLoopTimeUs = 0;

        return true;
    }

    // Reset specific task
    // Check in control tasks
    if (static_cast<size_t>(taskIndex) < m_controlTasks.size()) {
        m_controlTasks[taskIndex].executionTimeUs = 0;
        m_controlTasks[taskIndex].maxExecutionTimeUs = 0;
        m_controlTasks[taskIndex].missedDeadlines = 0;
        return true;
    }

    // Check in auxiliary tasks
    if (static_cast<size_t>(taskIndex) < m_auxiliaryTasks.size()) {
        m_auxiliaryTasks[taskIndex].executionTimeUs = 0;
        m_auxiliaryTasks[taskIndex].maxExecutionTimeUs = 0;
        m_auxiliaryTasks[taskIndex].missedDeadlines = 0;
        return true;
    }

    return false;
}

void TaskScheduler::getSchedulerStats(uint32_t& controlTaskCount, uint32_t& auxiliaryTaskCount,
                                      uint32_t& totalMissedDeadlines,
                                      uint32_t& averageControlLoopTimeUs) {
    controlTaskCount = m_controlTasks.size();
    auxiliaryTaskCount = m_auxiliaryTasks.size();
    totalMissedDeadlines = m_totalMissedDeadlines;

    // Calculate average control loop time
    if (m_totalExecutions > 0) {
        averageControlLoopTimeUs = m_totalExecutionTimeUs / m_totalExecutions;
    } else {
        averageControlLoopTimeUs = 0;
    }
}

bool TaskScheduler::isTaskDue(const TaskInfo& task, uint32_t currentTimeUs) {
    // Calculate time since last execution
    uint32_t elapsedUs;

    // Handle timer overflow
    if (currentTimeUs < task.lastExecutionUs) {
        elapsedUs = (UINT32_MAX - task.lastExecutionUs) + currentTimeUs + 1;
    } else {
        elapsedUs = currentTimeUs - task.lastExecutionUs;
    }

    // Check if interval has elapsed
    return elapsedUs >= task.intervalUs;
}

uint32_t TaskScheduler::executeTask(TaskInfo& task, uint32_t currentTimeUs) {
    // Record start time
    uint32_t startTimeUs = micros();

    // Execute task
    task.function();

    // Record end time
    uint32_t endTimeUs = micros();

    // Calculate execution time
    uint32_t executionTimeUs;

    // Handle timer overflow
    if (endTimeUs < startTimeUs) {
        executionTimeUs = (UINT32_MAX - startTimeUs) + endTimeUs + 1;
    } else {
        executionTimeUs = endTimeUs - startTimeUs;
    }

    // Update task statistics
    task.executionTimeUs = executionTimeUs;
    if (executionTimeUs > task.maxExecutionTimeUs) {
        task.maxExecutionTimeUs = executionTimeUs;
    }

    // Update scheduler statistics
    m_totalExecutions++;
    m_totalExecutionTimeUs += executionTimeUs;

    // Update last execution time
    task.lastExecutionUs = currentTimeUs;

    return executionTimeUs;
}