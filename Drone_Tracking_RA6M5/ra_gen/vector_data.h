/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (16)
#endif
/* ISR prototypes */
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 0) /* SCI9 RXI (Receive data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 0) /* SCI9 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
#define VECTOR_NUMBER_SCI5_RXI ((IRQn_Type) 4) /* SCI5 RXI (Receive data full) */
#define SCI5_RXI_IRQn          ((IRQn_Type) 4) /* SCI5 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI5_TXI ((IRQn_Type) 5) /* SCI5 TXI (Transmit data empty) */
#define SCI5_TXI_IRQn          ((IRQn_Type) 5) /* SCI5 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI5_TEI ((IRQn_Type) 6) /* SCI5 TEI (Transmit end) */
#define SCI5_TEI_IRQn          ((IRQn_Type) 6) /* SCI5 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI5_ERI ((IRQn_Type) 7) /* SCI5 ERI (Receive error) */
#define SCI5_ERI_IRQn          ((IRQn_Type) 7) /* SCI5 ERI (Receive error) */
#define VECTOR_NUMBER_SCI2_RXI ((IRQn_Type) 8) /* SCI2 RXI (Receive data full) */
#define SCI2_RXI_IRQn          ((IRQn_Type) 8) /* SCI2 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI2_TXI ((IRQn_Type) 9) /* SCI2 TXI (Transmit data empty) */
#define SCI2_TXI_IRQn          ((IRQn_Type) 9) /* SCI2 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI2_TEI ((IRQn_Type) 10) /* SCI2 TEI (Transmit end) */
#define SCI2_TEI_IRQn          ((IRQn_Type) 10) /* SCI2 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI2_ERI ((IRQn_Type) 11) /* SCI2 ERI (Receive error) */
#define SCI2_ERI_IRQn          ((IRQn_Type) 11) /* SCI2 ERI (Receive error) */
#define VECTOR_NUMBER_SCI3_RXI ((IRQn_Type) 12) /* SCI3 RXI (Receive data full) */
#define SCI3_RXI_IRQn          ((IRQn_Type) 12) /* SCI3 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI3_TXI ((IRQn_Type) 13) /* SCI3 TXI (Transmit data empty) */
#define SCI3_TXI_IRQn          ((IRQn_Type) 13) /* SCI3 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI3_TEI ((IRQn_Type) 14) /* SCI3 TEI (Transmit end) */
#define SCI3_TEI_IRQn          ((IRQn_Type) 14) /* SCI3 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI3_ERI ((IRQn_Type) 15) /* SCI3 ERI (Receive error) */
#define SCI3_ERI_IRQn          ((IRQn_Type) 15) /* SCI3 ERI (Receive error) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (16)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
