package org.firstinspires.ftc.teamcode.jankbot.util

import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.Supplier

class ActionQueue {
    private var actionQueue = arrayListOf<Pair<Runnable, Pair<Double, Supplier<Boolean>>>>()
    private var newActions = arrayListOf<Pair<Runnable, Pair<Double, Supplier<Boolean>>>>()
    private val timer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    private val trueFunc : Supplier<Boolean> = Supplier { true }

    private var internalTime = 0.0

    init {
        this.clearQueue()
    }

    /** @param delay: IN MILLISECONDS */
    fun addDelayedAction(action: Runnable, delay : Double) : ActionQueue {
        return this.addDelayedAction(action, delay, trueFunc)
    }

    /** @param delay: IN MILLISECONDS */
    fun addDelayedAction(action : Runnable, delay : Double, condition : Supplier<Boolean>) : ActionQueue {
        newActions.add(Pair(action, Pair(internalTime + delay, condition)))
        return this
    }

    fun addDelayedAction(action : Runnable, condition : Supplier<Boolean>) : ActionQueue {
        return this.addDelayedAction(action, 0.0, condition);
    }

    fun clearQueue() { actionQueue.clear()  }

    fun resetTimer() { timer.reset()  }

    /** Add a 1 second delayed action */
    fun addDelayedAction(action: Runnable) : ActionQueue {
        this.addDelayedAction(action,1000.0)
        return this
    }

    /** You should use the one with no params instead */
    fun update(time : Double) {
        internalTime = time
        actionQueue.addAll(newActions)
        newActions.clear()
        val actionIterator = actionQueue.listIterator()
        while (actionIterator.hasNext()){
            val a = actionIterator.next()
            if(a.second.first < time && a.second.second.get()) {
                a.first.run()
                actionIterator.remove()
            }
        }
    }

    fun update() { this.update(timer.milliseconds())  }

    fun getActionQueue() : ArrayList<Pair<Runnable, Pair<Double, Supplier<Boolean>>>> {
        return actionQueue;
    }
}