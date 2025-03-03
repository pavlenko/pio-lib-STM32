#pragma once

#include <stm32/_cmsis.hpp>

namespace STM32::Timer
{
    // All timer`s interrupts
    enum class IRQFlags
    {
        UPDATE = TIM_DIER_UIE,
        CC1 = TIM_DIER_CC1IE,
        CC2 = TIM_DIER_CC2IE,
        CC3 = TIM_DIER_CC3IE,
        CC4 = TIM_DIER_CC4IE,
        COM = TIM_DIER_COMIE,
        TRIGGER = TIM_DIER_TIE,
        BREAK = TIM_DIER_BIE,
    };

    // All timer`s DMA requests
    enum class DMAFlags
    {
        UPDATE = TIM_DIER_UDE,
        CC1 = TIM_DIER_CC1DE,
        CC2 = TIM_DIER_CC2DE,
        CC3 = TIM_DIER_CC3DE,
        CC4 = TIM_DIER_CC4DE,
        TRIGGER = TIM_DIER_UDE,
    };

    // Timer counter mode
    enum class CounterMode
    {
        // Direction
        UP = 0x00000000U,
        DOWN = TIM_CR1_DIR,
        // One-pulse mode
        ONE_PULSE = TIM_CR1_OPM,
        // Center-aligned mode
        CENTER_ALIGNED1 = TIM_CR1_CMS_0,
        CENTER_ALIGNED2 = TIM_CR1_CMS_1,
        CENTER_ALIGNED3 = TIM_CR1_CMS,
    };

    // Timer master mode
    enum class MasterMode
    {
        RESET = 0x0 << TIM_CR2_MMS_Pos,///< Reset is used as TRGO
        ENABLE = 0x1 << TIM_CR2_MMS_Pos,///< Counter enable is used as TRGO
        UPDATE = 0x2 << TIM_CR2_MMS_Pos,///< Update event is used as TRGO
        ComparePulse = 0x3 << TIM_CR2_MMS_Pos,///< CC1F set is used as TRGO
        CompareCh1 = 0x4 << TIM_CR2_MMS_Pos,///< OC1REF signal is used as TRGO
        CompareCh2 = 0x5 << TIM_CR2_MMS_Pos,///< OC2REF signal is used as TRGO
        CompareCh3 = 0x6 << TIM_CR2_MMS_Pos,///< OC3REF signal is used as TRGO
        CompareCh4 = 0x7 << TIM_CR2_MMS_Pos,///< OC4REF signal is used as TRGO
    };

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    class BasicTimer
    {
        template <typename tConfig>
        static inline void configure();
        static inline void setPrescaller(uint16_t prescaller);
        static inline void setAutoReload(uint16_t autoReload);
        static inline void setCounter(uint16_t conter);
        static inline void enable();
        static inline void disable();
        static inline void attachIRQ(IRQFlags flags);
        static inline void detachIRQ(IRQFlags flags);
        static inline void attachDMARequest();
        static inline void detachDMARequest();
        static inline bool hasIRQFlag();
        static inline void clrIRQFlag();
        static inline void start();
        static inline void stop();
    };

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    class GPTimer : public BasicTimer<tRegsAddr, tIRQn, tClock>
    {
    private:
        template <uint8_t tNumber>
        class Channel
        {
        public:
            static inline void enable();
            static inline void disable();
            static inline bool hasIRQFlag();
            static inline void clrIRQFlag();
            static inline void attachIRQ();
            static inline void detachIRQ();
            static inline void attachDMARequest();
            static inline void detachDMARequest();
        };

    public:
        template <uint8_t tNumber>
        class ICapture : public Channel<tNumber>
        {
        public:
            // Capture polarity
            enum class Polarity
            {
                RISING = 0,
                FALLING = TIM_CCER_CC1P,
                BOTH = TIM_CCER_CC1P | TIM_CCER_CC1NP,
            };

            // Capture mode
            enum class Mode
            {
                DIRECT = TIM_CCMR1_CC1S_0,
                INDIRECT = TIM_CCMR1_CC1S_1,
                TRC = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1,
            };

            static inline void configure();
        };

        template <uint8_t tNumber>
        class OCompare : public Channel<tNumber>
        {
        public:
            // Output polarity
            enum Polarity
            {
                HIGH = 0,
                LOW  = TIM_CCER_CC1P,
            };

            // Output mode
            enum Mode
            {
                TIMING          = 0,
                ACTIVE          = TIM_CCMR1_OC1M_0,
                INACTIVE        = TIM_CCMR1_OC1M_1,
                TOGGLE          = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1,
                PWM1            = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2,
                PWM2            = TIM_CCMR1_OC1M,
                FORCED_ACTIVE   = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2,
                FORCED_INACTIVE = TIM_CCMR1_OC1M_2,
            };

            static inline void configure();
        };

        template <uint8_t tNumber>
        class PWMGeneration : public OCompare<tNumber>
        {
        public:
            // PWM Fast mod
            enum class FastMode
            {
                DISABLE = 0x00000000U,
                ENABLE  = TIM_CCMR1_OC1FE,
            };

            static inline void configure();
        };

        class SlaveMode //TODO incomplete... need read docs
        {
        public:
            // TODO use single enum for full config options
            // Slave mode selection
            enum class Mode : uint16_t
            {
                DISABLED = 0x00 << TIM_SMCR_SMS_Pos,
                ENCODER_MODE1 = 0x01 << TIM_SMCR_SMS_Pos,
                ENCODER_MODE2 = 0x02 << TIM_SMCR_SMS_Pos,
                ENCODER_MODE3 = 0x03 << TIM_SMCR_SMS_Pos,
                RESET = 0x04 << TIM_SMCR_SMS_Pos,
                GATED = 0x05 << TIM_SMCR_SMS_Pos,
                TRIGGER = 0x06 << TIM_SMCR_SMS_Pos,
                EXTERNAL_CLOCK = 0x07 << TIM_SMCR_SMS_Pos,
            };

            /// Trigger selection
            enum class Trigger : uint16_t
            {
                INTERNAL_TRIGGER0 = 0x00 << TIM_SMCR_TS_Pos,
                INTERNAL_TRIGGER1 = 0x01 << TIM_SMCR_TS_Pos,
                INTERNAL_TRIGGER2 = 0x02 << TIM_SMCR_TS_Pos,
                INTERNAL_TRIGGER3 = 0x03 << TIM_SMCR_TS_Pos,
                Ti1EdgeDetector = 0x04 << TIM_SMCR_TS_Pos,
                FilteredTimerInput1 = 0x05 << TIM_SMCR_TS_Pos,
                FilteredTimerInput2 = 0x06 << TIM_SMCR_TS_Pos,
                ExternalTriggerInput = 0x07 << TIM_SMCR_TS_Pos,
            };

            /// External trigger filter selection
            enum class ExternalTriggerFilter : uint16_t
            {
                NoFilter = 0x00 << TIM_SMCR_ETF_Pos,
                NoDivideFilter2 = 0x01 << TIM_SMCR_ETF_Pos,
                NoDivideFilter4 = 0x02 << TIM_SMCR_ETF_Pos,
                NoDivideFilter8 = 0x03 << TIM_SMCR_ETF_Pos,
                Divide2Filter6 = 0x04 << TIM_SMCR_ETF_Pos,
                Divide2Filter8 = 0x05 << TIM_SMCR_ETF_Pos,
                Divide4Filter6 = 0x06 << TIM_SMCR_ETF_Pos,
                Divide4Filter8 = 0x07 << TIM_SMCR_ETF_Pos,
                Divide8Filter6 = 0x08 << TIM_SMCR_ETF_Pos,
                Divide8Filter8 = 0x09 << TIM_SMCR_ETF_Pos,
                Divide16Filter5 = 0x0a << TIM_SMCR_ETF_Pos,
                Divide16Filter6 = 0x0b << TIM_SMCR_ETF_Pos,
                Divide16Filter8 = 0x0c << TIM_SMCR_ETF_Pos,
                Divide32Filter5 = 0x0d << TIM_SMCR_ETF_Pos,
                Divide32Filter6 = 0x0e << TIM_SMCR_ETF_Pos,
                Divide32Filter8 = 0x0f << TIM_SMCR_ETF_Pos,
            };

            /// External trigger filter selection
            enum class ExternalTriggerPrescaler : uint16_t
            {
                PrescalerOff = 0x00 << TIM_SMCR_ETPS_Pos,
                Divide2 = 0x01 << TIM_SMCR_ETPS_Pos,
                Divide4 = 0x02 << TIM_SMCR_ETPS_Pos,
                Divide8 = 0x03 << TIM_SMCR_ETPS_Pos,
            };

            /// External clock mode 2 enable
            enum class ExternalClockMode2 : uint16_t
            {
                Disabled = 0x00 << TIM_SMCR_ECE_Pos,
                Enabled = 0x01 << TIM_SMCR_ECE_Pos,
            };

            /// External trigger polarity
            enum class ExternalTriggerPolarity : uint16_t
            {
                NonInverted = 0x00 << TIM_SMCR_ETP_Pos,
                Inverted = 0x01 << TIM_SMCR_ETP_Pos,
            };

        };
    };

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    class AdvancedTimer : public GPTimer<tRegsAddr, tIRQn, tClock>
    {
    public:
        static inline void setRepetitionCounter(uint8_t counter);
        static inline uint8_t getRepetitionCounter();
    };
}
