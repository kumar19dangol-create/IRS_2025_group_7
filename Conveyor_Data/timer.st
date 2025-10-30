PROGRAM timer
  VAR
    weight AT %MW0 : INT := 0;
  END_VAR
  VAR
    bigbox : INT := 10;
    midbox1 : INT := 5;
    midbox0 : INT := 10;
    smlbox : INT := 5;
    smlbox0 : INT := 0;
  END_VAR
  VAR
    btn AT %QX0.0 : BOOL := False;
    mtr AT %QX0.1 : BOOL := False;
  END_VAR
  VAR
    TON0 : TON;
    Bigstop : BOOL := True;
    Midstop : BOOL := True;
    Smlstop : BOOL := True;
    bigtimer : TIME := T#2s;
    midtimer : TIME := T#3.5s;
    smltimer : TIME := T#5.1s;
    CTU0 : CTU;
  END_VAR
  VAR
    Pulse AT %QX0.2 : BOOL := False;
    reset AT %QX0.3 : BOOL := False;
  END_VAR
  VAR
    preset : INT := 100;
  END_VAR
  VAR
    BigCnt AT %MW2 : INT := 0;
    MidCnt AT %MW3 : INT := 0;
    SmlCnt AT %MW4 : INT := 0;
    TotalCnt AT %MW1 : INT := 0;
  END_VAR
  VAR
    TON1 : TON;
    CTU1 : CTU;
    CTU2 : CTU;
    TON2 : TON;
    _TMP_GT4_OUT : BOOL;
    R_TRIG1 : R_TRIG;
    _TMP_ADD54_OUT : INT;
    _TMP_OR58_OUT : BOOL;
    _TMP_NOT23_OUT : BOOL;
    _TMP_GT25_OUT : BOOL;
    _TMP_LE26_OUT : BOOL;
    _TMP_AND27_OUT : BOOL;
    R_TRIG2 : R_TRIG;
    _TMP_GT60_OUT : BOOL;
    _TMP_LE40_OUT : BOOL;
    _TMP_AND59_OUT : BOOL;
    R_TRIG3 : R_TRIG;
  END_VAR

  mtr := btn;
  _TMP_GT4_OUT := GT(weight, bigbox);
  TON0(IN := _TMP_GT4_OUT, PT := bigtimer);
  Bigstop := TON0.Q;
  R_TRIG1(CLK := Pulse AND _TMP_GT4_OUT);
  CTU0(CU := R_TRIG1.Q, R := reset, PV := preset);
  BigCnt := CTU0.CV;
  _TMP_ADD54_OUT := ADD(BigCnt, MidCnt, SmlCnt);
  TotalCnt := _TMP_ADD54_OUT;
  _TMP_OR58_OUT := OR(Bigstop, Midstop, Smlstop);
  _TMP_NOT23_OUT := NOT(_TMP_OR58_OUT);
  mtr := _TMP_NOT23_OUT;
  _TMP_GT25_OUT := GT(weight, midbox1);
  _TMP_LE26_OUT := LE(weight, midbox0);
  _TMP_AND27_OUT := AND(_TMP_GT25_OUT, _TMP_LE26_OUT);
  R_TRIG2(CLK := Pulse AND _TMP_AND27_OUT);
  CTU1(CU := R_TRIG2.Q, R := reset, PV := preset);
  MidCnt := CTU1.CV;
  TON1(IN := _TMP_AND27_OUT, PT := midtimer);
  Midstop := TON1.Q;
  _TMP_GT60_OUT := GT(weight, smlbox0);
  _TMP_LE40_OUT := LE(weight, smlbox);
  _TMP_AND59_OUT := AND(_TMP_GT60_OUT, _TMP_LE40_OUT);
  R_TRIG3(CLK := Pulse AND _TMP_AND59_OUT);
  CTU2(CU := R_TRIG3.Q, R := reset, PV := preset);
  SmlCnt := CTU2.CV;
  TON2(IN := _TMP_AND59_OUT, PT := smltimer);
  Smlstop := TON2.Q;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 0);
    PROGRAM instance0 WITH task0 : timer;
  END_RESOURCE
END_CONFIGURATION

(*DBG:char md5[] = "79569e933deda33f27b2d4f620570993";*)
(*DBG:/**)
(*DBG: * This file is part of OpenPLC Runtime*)
(*DBG: **)
(*DBG: * Copyright (C) 2023 Autonomy, GP Orcullo*)
(*DBG: * Based on the work by GP Orcullo on Beremiz for uC*)
(*DBG: **)
(*DBG: * This program is free software; you can redistribute it and/or*)
(*DBG: * modify it under the terms of the GNU General Public License*)
(*DBG: * as published by the Free Software Foundation; either version 2*)
(*DBG: * of the License, or (at your option) any later version.*)
(*DBG: **)
(*DBG: * This program is distributed in the hope that it will be useful,*)
(*DBG: * but WITHOUT ANY WARRANTY; without even the implied warranty of*)
(*DBG: * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the*)
(*DBG: * GNU General Public License for more details.*)
(*DBG: **)
(*DBG: * You should have received a copy of the GNU General Public License*)
(*DBG: * along with this program; If not, see <http://www.gnu.org/licenses/>.*)
(*DBG: **)
(*DBG: */*)
(*DBG:*)
(*DBG:#include <stdbool.h>*)
(*DBG:*)
(*DBG:#include "iec_types_all.h"*)
(*DBG:#include "POUS.h"*)
(*DBG:*)
(*DBG:#define SAME_ENDIANNESS      0*)
(*DBG:#define REVERSE_ENDIANNESS   1*)
(*DBG:*)
(*DBG:uint8_t endianness;*)
(*DBG:*)
(*DBG:*)
(*DBG:extern TIMER RES0__INSTANCE0;*)
(*DBG:*)
(*DBG:static const struct {*)
(*DBG:    void *ptr;*)
(*DBG:    __IEC_types_enum type;*)
(*DBG:} debug_vars[] = {*)
(*DBG:    {&(RES0__INSTANCE0.WEIGHT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOX), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MIDBOX1), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MIDBOX0), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMLBOX), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMLBOX0), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BTN), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MTR), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGSTOP), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MIDSTOP), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMLSTOP), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGTIMER), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MIDTIMER), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMLTIMER), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PULSE), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.RESET), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PRESET), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MIDCNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMLCNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TOTALCNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT4_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_ADD54_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_OR58_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_NOT23_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT25_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LE26_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND27_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT60_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LE40_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND59_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.M), BOOL_ENUM},*)
(*DBG:};*)
(*DBG:*)
(*DBG:#define VAR_COUNT               112*)
(*DBG:*)
(*DBG:uint16_t get_var_count(void)*)
(*DBG:{*)
(*DBG:    return VAR_COUNT;*)
(*DBG:}*)
(*DBG:*)
(*DBG:size_t get_var_size(size_t idx)*)
(*DBG:{*)
(*DBG:    if (idx >= VAR_COUNT)*)
(*DBG:    {*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case TIME_ENUM:*)
(*DBG:        return sizeof(TIME);*)
(*DBG:    case INT_ENUM:*)
(*DBG:    case INT_O_ENUM:*)
(*DBG:        return sizeof(INT);*)
(*DBG:    case SINT_ENUM:*)
(*DBG:        return sizeof(SINT);*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:    case BOOL_O_ENUM:*)
(*DBG:        return sizeof(BOOL);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void *get_var_addr(size_t idx)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case TIME_ENUM:*)
(*DBG:        return (void *)&((__IEC_TIME_t *) ptr)->value;*)
(*DBG:    case INT_ENUM:*)
(*DBG:        return (void *)&((__IEC_INT_t *) ptr)->value;*)
(*DBG:    case INT_O_ENUM:*)
(*DBG:        return (void *)((((__IEC_INT_p *) ptr)->flags & __IEC_FORCE_FLAG) *)
(*DBG:                        ? &(((__IEC_INT_p *) ptr)->fvalue) *)
(*DBG:                        : ((__IEC_INT_p *) ptr)->value);*)
(*DBG:    case SINT_ENUM:*)
(*DBG:        return (void *)&((__IEC_SINT_t *) ptr)->value;*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:        return (void *)&((__IEC_BOOL_t *) ptr)->value;*)
(*DBG:    case BOOL_O_ENUM:*)
(*DBG:        return (void *)((((__IEC_BOOL_p *) ptr)->flags & __IEC_FORCE_FLAG) *)
(*DBG:                        ? &(((__IEC_BOOL_p *) ptr)->fvalue) *)
(*DBG:                        : ((__IEC_BOOL_p *) ptr)->value);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void force_var(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    if (forced) {*)
(*DBG:        size_t var_size = get_var_size(idx);*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case TIME_ENUM: {*)
(*DBG:            memcpy(&((__IEC_TIME_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_TIME_t *) ptr)->value = *((TIME *) val);*)
(*DBG:            ((__IEC_TIME_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_INT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_INT_t *) ptr)->value = *((INT *) val);*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_O_ENUM: {*)
(*DBG:            memcpy((((__IEC_INT_p *) ptr)->value), val, var_size);*)
(*DBG:            //*(((__IEC_INT_p *) ptr)->value) = *((INT *) val);*)
(*DBG:            ((__IEC_INT_p *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case SINT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_SINT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_SINT_t *) ptr)->value = *((SINT *) val);*)
(*DBG:            ((__IEC_SINT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_ENUM: {*)
(*DBG:            memcpy(&((__IEC_BOOL_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_BOOL_t *) ptr)->value = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_O_ENUM: {*)
(*DBG:            memcpy((((__IEC_BOOL_p *) ptr)->value), val, var_size);*)
(*DBG:            //*(((__IEC_BOOL_p *) ptr)->value) = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_p *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    } else {*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case TIME_ENUM:*)
(*DBG:            ((__IEC_TIME_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_ENUM:*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_O_ENUM:*)
(*DBG:            ((__IEC_INT_p *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case SINT_ENUM:*)
(*DBG:            ((__IEC_SINT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_ENUM:*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_O_ENUM:*)
(*DBG:            ((__IEC_BOOL_p *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void swap_bytes(void *ptr, size_t size) *)
(*DBG:{*)
(*DBG:    uint8_t *bytePtr = (uint8_t *)ptr;*)
(*DBG:    size_t i;*)
(*DBG:    for (i = 0; i < size / 2; ++i) *)
(*DBG:    {*)
(*DBG:        uint8_t temp = bytePtr[i];*)
(*DBG:        bytePtr[i] = bytePtr[size - 1 - i];*)
(*DBG:        bytePtr[size - 1 - i] = temp;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void trace_reset(void)*)
(*DBG:{*)
(*DBG:    for (size_t i=0; i < VAR_COUNT; i++) *)
(*DBG:    {*)
(*DBG:        force_var(i, false, 0);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_trace(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    if (idx >= 0 && idx < VAR_COUNT) *)
(*DBG:    {*)
(*DBG:        if (endianness == REVERSE_ENDIANNESS)*)
(*DBG:        {*)
(*DBG:            // Aaaaarghhhh... Stupid AVR is Big Endian.*)
(*DBG:            swap_bytes(val, get_var_size(idx));*)
(*DBG:        }*)
(*DBG:*)
(*DBG:        force_var(idx, forced, val);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_endianness(uint8_t value)*)
(*DBG:{*)
(*DBG:    if (value == SAME_ENDIANNESS || value == REVERSE_ENDIANNESS)*)
(*DBG:    {*)
(*DBG:        endianness = value;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)