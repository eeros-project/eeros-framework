#ifndef ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#define ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_

#include <vector>
#include <mutex>
#include <eeros/core/Runnable.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetyContext.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
  namespace safety {
    /**
     * \brief Safety system.
     *
     * Every robot control system needs exactly one safety system. The safety system comprises several safety levels and safety events to switch between these levels
     */
    class SafetySystem : public Runnable {
    public:
      /**
       * Creates a safety system
       * @param properties Safety properties belonging to this safety system.
       * @param period The period determines the speed, which the executor will run the safety system.
       */
      SafetySystem(SafetyProperties& properties, double period);

      /**
       * Destructor, do not call manually.
       */
      virtual ~SafetySystem();

      /**
       * Getter function for the current safety level.
       * @return The current safety level.
       */
      SafetyLevel& getCurrentLevel(void);

      /**
       * Causes a fire a safety event. Make sure that this event is registered for the actual safety level
       * and that it is a public event in case of being fired from outside of the safety system.
       * @param event The safety event to be fired.
       * @param context The context, in which the event is fired, this could be public or private.
       */
      void triggerEvent(SafetyEvent event, SafetyContext* context = nullptr);

      /**
       * Getter function for the current safety properties.
       * @return The current safety properties.
       */
      const SafetyProperties* getProperties() const;

      /**
       * Getter function for the period value.
       * @return The period of the safety system.
       */
      double getPeriod() const;

      /**
       * The executor will call this method when the safety system has to run. Do not call manually.
       */
      void run();

      /**
       * This method can be used to shut the system down gracefully when stopping a program with signals, e.g. SIGINT.
       * Register a signal handler and call this function from there.
       */
      static void exitHandler();

      /**
       * This logger is used to put out information about the safety system
       */
      logger::Logger log;

    private:
      bool setProperties(SafetyProperties& safetyProperties);
      std::mutex mtx;
      SafetyProperties properties;
      SafetyLevel* currentLevel;
      SafetyLevel* nextLevel;
      SafetyContext privateContext;
      static uint8_t instCount;
      static SafetySystem* instance;
      double period;
    };

  };
};

#endif // ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
