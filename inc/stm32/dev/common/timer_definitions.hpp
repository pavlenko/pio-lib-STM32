#pragma once

#include <stm32/dev/common/_cmsis.hpp>

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
    protected:
        /**
         * @brief Access registers
         */
        static inline TIM_TypeDef *_regs();

    public:
        static inline void setCounterMode(CounterMode mode);
        static inline void setMasterMode(MasterMode mode);

        /**
         * @brief Set prescaler
         */
        static inline void setPrescaler(uint16_t prescaler);

        /**
         * @brief Set auto-reload value
         */
        static inline void setAutoReload(uint16_t autoReload);

        /**
         * @brief Set counter value
         */
        static inline void setCounter(uint16_t counter);

        /**
         * @brief Enable timer clock
         */
        static inline void enable();

        /**
         * @brief Disable timer clock
         */
        static inline void disable();

        /**
         * @brief Enable timer interrupts
         */
        static inline void attachIRQ(IRQFlags flags);

        /**
         * @brief Disable timer interrupts
         */
        static inline void detachIRQ(IRQFlags flags);

        /**
         * @brief Check interrupt flag
         */
        static inline bool hasIRQFlag();

        /**
         * @brief Clear interrupt flag
         */
        static inline void clrIRQFlag();

        /**
         * @brief Attach DMA
         */
        static inline void attachDMARequest();

        /**
         * @brief Detach DMA
         */
        static inline void detachDMARequest();

        /**
         * @brief Start timer
         */
        static inline void start();

        /**
         * @brief Stop timer
         */
        static inline void stop();
    };

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, uint8_t tChannels>
    class GPTimer : public BasicTimer<tRegsAddr, tIRQn, tClock>
    {
    private:
        template <uint8_t tNumber>
        class Channel
        {
        protected:
            static_assert(tNumber < tChannels, "Invalid channel number");

            static constexpr const uint8_t _4bit_pos = tNumber * 4u;
            static constexpr const uint8_t _8bit_pos = (tNumber & 0x1u) * 8u;

        public:
            /**
             * @brief Enable channel ouput
             */
            static inline void enable();

            /**
             * @brief Disable channel output
             */
            static inline void disable();

            /**
             * @brief Check channel interrupt flag
             */
            static inline bool hasIRQFlag();

            /**
             * @brief Clear channel interrupt flag
             */
            static inline void clrIRQFlag();

            /**
             * @brief Enable channel interrupt
             */
            static inline void attachIRQ();

            /**
             * @brief Disable channel interrupt
             */
            static inline void detachIRQ();

            /**
             * @brief Enable channel DMA
             */
            static inline void attachDMARequest();

            /**
             * @brief Disable channel DMA
             */
            static inline void detachDMARequest();
        };

    public:
        template <uint8_t tNumber>
        class ICapture : public Channel<tNumber>
        {
        public:
            /**
             * @brief Capture polarity
             */
            enum class Polarity : uint32_t
            {
                RISING = 0,
                FALLING = TIM_CCER_CC1P,
                BOTH = TIM_CCER_CC1P | TIM_CCER_CC1NP,
            };

            /**
             * @brief Capture mode
             */
            enum class Mode : uint32_t
            {
                DIRECT = TIM_CCMR1_CC1S_0,
                INDIRECT = TIM_CCMR1_CC1S_1,
                TRC = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1,
            };

            /**
             * @brief Set capture polarity
             */
            static inline void setPolarity(Polarity polarity);

            /**
             * @brief Set capture mode
             */
            static inline void setMode(Mode mode);

            /**
             * @brief Get captured value
             */
            static inline uint16_t getValue();
        };

        template <uint8_t tNumber>
        class OCompare : public Channel<tNumber>
        {
        public:
            /**
             * @brief Output polarity
             */
            enum Polarity : uint32_t
            {
                HIGH = 0,
                LOW  = TIM_CCER_CC1P,
            };

            /**
             * @brief Output mode
             */
            enum Mode : uint32_t
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

            /**
             * @brief Set compare polarity
             */
            static inline void setPolarity(Polarity polarity);

            /**
             * @brief Set compare mode
             */
            static inline void setMode(Mode mode);

            /**
             * @brief Set compare pulse
             */
            static inline void setPulse(uint16_t pulse);

            /**
             * @brief Get compare pulse
             */
            static inline uint16_t getPulse();
        };

        template <uint8_t tNumber>
        class PWMGeneration : public OCompare<tNumber>
        {
        public:
            /**
             * @brief PWM Fast mode enabled
             */
            enum class FastMode
            {
                DISABLE = 0x00000000U,
                ENABLE  = TIM_CCMR1_OC1FE,
            };

            /**
             * @brief Set fast mode enabled
             */
            static inline void setFastMode(FastMode mode);
        };

        class SlaveMode //TODO incomplete... need readdocs
        {
        public:
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
                INPUT1_EDGE_DETECT = 0x04 << TIM_SMCR_TS_Pos,
                FILTERED_INPUT1 = 0x05 << TIM_SMCR_TS_Pos,
                FILTERED_INPUT2 = 0x06 << TIM_SMCR_TS_Pos,
                EXTERNAL_INPUT = 0x07 << TIM_SMCR_TS_Pos,
            };

            /// External trigger filter selection
            enum class ExternalTriggerFilter : uint16_t
            {
                NO_FILTER = 0x00 << TIM_SMCR_ETF_Pos,
                N2 = 0x01 << TIM_SMCR_ETF_Pos,
                N4 = 0x02 << TIM_SMCR_ETF_Pos,
                N8 = 0x03 << TIM_SMCR_ETF_Pos,
                DIV2_N6 = 0x04 << TIM_SMCR_ETF_Pos,
                DIV2_N8 = 0x05 << TIM_SMCR_ETF_Pos,
                DIV4_N6 = 0x06 << TIM_SMCR_ETF_Pos,
                DIV4_N8 = 0x07 << TIM_SMCR_ETF_Pos,
                DIV8_N6 = 0x08 << TIM_SMCR_ETF_Pos,
                DIV8_N8 = 0x09 << TIM_SMCR_ETF_Pos,
                DIV16_N5 = 0x0a << TIM_SMCR_ETF_Pos,
                DIV16_N6 = 0x0b << TIM_SMCR_ETF_Pos,
                DIV16_N8 = 0x0c << TIM_SMCR_ETF_Pos,
                DIV32_N5 = 0x0d << TIM_SMCR_ETF_Pos,
                DIV32_N6 = 0x0e << TIM_SMCR_ETF_Pos,
                DIV32_N8 = 0x0f << TIM_SMCR_ETF_Pos,
            };

            /// External trigger filter selection
            enum class ExternalTriggerPrescaler : uint16_t
            {
                OFF = 0x00 << TIM_SMCR_ETPS_Pos,
                DIV2 = 0x01 << TIM_SMCR_ETPS_Pos,
                DIV4 = 0x02 << TIM_SMCR_ETPS_Pos,
                DIV8 = 0x03 << TIM_SMCR_ETPS_Pos,
            };

            /// External clock mode 2 enable
            enum class ExternalClockMode2 : uint16_t
            {
                DISABLED = 0x00 << TIM_SMCR_ECE_Pos,
                ENABLED = 0x01 << TIM_SMCR_ECE_Pos,
            };

            /// External trigger polarity
            enum class ExternalTriggerPolarity : uint16_t
            {
                NORMAL = 0x00 << TIM_SMCR_ETP_Pos,
                INVERTED = 0x01 << TIM_SMCR_ETP_Pos,
            };

            //TODO methods: enable(mode),disable(),setTrigger(),setTriggerFilter(),setTriggerPolarity(),setTriggerPrescaler()
        };
    };

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, uint8_t tChannels>
    class AdvancedTimer : public GPTimer<tRegsAddr, tIRQn, tClock, tChannels>
    {
    public:
        /**
         * @brief Set repetition counter value
         */
        static inline void setRepetitionCounter(uint8_t counter);

        /**
         * @brief Get repetition counter value
         */
        static inline uint8_t getRepetitionCounter();
    };
}
