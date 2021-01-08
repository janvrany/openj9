/*******************************************************************************
 * Copyright (c) 2021, 2021 IBM Corp. and others
 *
 * This program and the accompanying materials are made available under
 * the terms of the Eclipse Public License 2.0 which accompanies this
 * distribution and is available at https://www.eclipse.org/legal/epl-2.0/
 * or the Apache License, Version 2.0 which accompanies this distribution and
 * is available at https://www.apache.org/licenses/LICENSE-2.0.
 *
 * This Source Code may also be made available under the following
 * Secondary Licenses when the conditions for such availability set
 * forth in the Eclipse Public License, v. 2.0 are satisfied: GNU
 * General Public License, version 2 with the GNU Classpath
 * Exception [1] and GNU General Public License, version 2 with the
 * OpenJDK Assembly Exception [2].
 *
 * [1] https://www.gnu.org/software/classpath/license.html
 * [2] http://openjdk.java.net/legal/assembly-exception.html
 *
 * SPDX-License-Identifier: EPL-2.0 OR Apache-2.0 OR GPL-2.0 WITH Classpath-exception-2.0 OR LicenseRef-GPL-2.0 WITH Assembly-exception
 *******************************************************************************/

#include <algorithm>
#include <iterator>

#include "codegen/RVInstruction.hpp"
#include "codegen/RVPrivateLinkage.hpp"
#include "codegen/RVSystemLinkage.hpp"
#include "codegen/CallSnippet.hpp"
#include "codegen/CodeGenerator.hpp"
#include "codegen/CodeGeneratorUtils.hpp"
#include "codegen/GCStackAtlas.hpp"
#include "codegen/GenerateInstructions.hpp"
#include "codegen/Linkage_inlines.hpp"
#include "codegen/Machine.hpp"
#include "codegen/MemoryReference.hpp"
#include "codegen/RealRegister.hpp"
#include "codegen/Register.hpp"
#include "codegen/StackCheckFailureSnippet.hpp"
#include "compile/Compilation.hpp"
#include "env/CompilerEnv.hpp"
#include "env/J2IThunk.hpp"
#include "env/StackMemoryRegion.hpp"
#include "exceptions/JITShutDown.hpp"
#include "il/Node_inlines.hpp"
#include "il/ParameterSymbol.hpp"
#include "il/ResolvedMethodSymbol.hpp"
#include "il/SymbolReference.hpp"
#include "infra/Assert.hpp"
#include "infra/List.hpp"

J9::RV::PrivateLinkageProperties::PrivateLinkageProperties()
   {
   _properties = IntegersInRegisters|FloatsInRegisters|RightToLeft;

   /*
    * _registerFlags for each register are defined in architectural order,
    * that is, from x0 to x31, f0 to f31.
    *
    * See https://github.com/riscv/riscv-elf-psabi-doc/blob/master/riscv-elf.md#integer-register-convention
    */
   _properties = IntegersInRegisters|FloatsInRegisters|RightToLeft;

   _registerFlags[TR::RealRegister::zero] = Preserved|RV_Reserved; // zero
   _registerFlags[TR::RealRegister::ra]   = Preserved|RV_Reserved; // return address
   _registerFlags[TR::RealRegister::sp]   = Preserved|RV_Reserved; // sp
   _registerFlags[TR::RealRegister::gp]   = Preserved|RV_Reserved; // gp
   _registerFlags[TR::RealRegister::tp]   = Preserved|RV_Reserved; // tp

   _registerFlags[TR::RealRegister::t0]   = Preserved|RV_Reserved; // fp
   _registerFlags[TR::RealRegister::t1]   = 0;
   _registerFlags[TR::RealRegister::t2]   = 0;

   _registerFlags[TR::RealRegister::s0]   = Preserved;
   _registerFlags[TR::RealRegister::s1]   = Preserved;

   _registerFlags[TR::RealRegister::a0]   = IntegerArgument | IntegerReturn;
   _registerFlags[TR::RealRegister::a1]   = IntegerArgument | IntegerReturn;
   _registerFlags[TR::RealRegister::a2]   = IntegerArgument;
   _registerFlags[TR::RealRegister::a3]   = IntegerArgument;
   _registerFlags[TR::RealRegister::a4]   = IntegerArgument;
   _registerFlags[TR::RealRegister::a5]   = IntegerArgument;
   _registerFlags[TR::RealRegister::a6]   = IntegerArgument;
   _registerFlags[TR::RealRegister::a7]   = IntegerArgument;

   _registerFlags[TR::RealRegister::s2]   = Preserved;
   _registerFlags[TR::RealRegister::s3]   = Preserved;
   _registerFlags[TR::RealRegister::s4]   = Preserved;
   _registerFlags[TR::RealRegister::s5]   = Preserved;
   _registerFlags[TR::RealRegister::s6]   = Preserved;
   _registerFlags[TR::RealRegister::s7]   = Preserved;
   _registerFlags[TR::RealRegister::s8]   = Preserved;
   _registerFlags[TR::RealRegister::s9]   = Preserved;
   _registerFlags[TR::RealRegister::s10]  = Preserved|RV_Reserved; // vmThread
   _registerFlags[TR::RealRegister::s11]  = Preserved|RV_Reserved; // Java SP

   _registerFlags[TR::RealRegister::t3]   = 0;
   _registerFlags[TR::RealRegister::t4]   = 0;
   _registerFlags[TR::RealRegister::t5]   = 0;
   _registerFlags[TR::RealRegister::t6]   = 0;

   _registerFlags[TR::RealRegister::ft0]  = 0;
   _registerFlags[TR::RealRegister::ft1]  = 0;
   _registerFlags[TR::RealRegister::ft2]  = 0;
   _registerFlags[TR::RealRegister::ft3]  = 0;
   _registerFlags[TR::RealRegister::ft4]  = 0;
   _registerFlags[TR::RealRegister::ft5]  = 0;
   _registerFlags[TR::RealRegister::ft6]  = 0;
   _registerFlags[TR::RealRegister::ft7]  = 0;

   _registerFlags[TR::RealRegister::fs0]  = Preserved;
   _registerFlags[TR::RealRegister::fs1]  = Preserved;

   _registerFlags[TR::RealRegister::fa0]  = FloatArgument | FloatReturn;
   _registerFlags[TR::RealRegister::fa1]  = FloatArgument | FloatReturn;
   _registerFlags[TR::RealRegister::fa2]  = FloatArgument;
   _registerFlags[TR::RealRegister::fa3]  = FloatArgument;
   _registerFlags[TR::RealRegister::fa4]  = FloatArgument;
   _registerFlags[TR::RealRegister::fa5]  = FloatArgument;
   _registerFlags[TR::RealRegister::fa6]  = FloatArgument;
   _registerFlags[TR::RealRegister::fa7]  = FloatArgument;

   _registerFlags[TR::RealRegister::fs2]  = Preserved;
   _registerFlags[TR::RealRegister::fs3]  = Preserved;
   _registerFlags[TR::RealRegister::fs4]  = Preserved;
   _registerFlags[TR::RealRegister::fs5]  = Preserved;
   _registerFlags[TR::RealRegister::fs6]  = Preserved;
   _registerFlags[TR::RealRegister::fs7]  = Preserved;
   _registerFlags[TR::RealRegister::fs8]  = Preserved;
   _registerFlags[TR::RealRegister::fs9]  = Preserved;
   _registerFlags[TR::RealRegister::fs10] = Preserved;
   _registerFlags[TR::RealRegister::fs11] = Preserved;
   _registerFlags[TR::RealRegister::ft8]  = 0;
   _registerFlags[TR::RealRegister::ft9]  = 0;
   _registerFlags[TR::RealRegister::ft10] = 0;
   _registerFlags[TR::RealRegister::ft11] = 0;


   _methodMetaDataRegister      = TR::RealRegister::s10;
   _stackPointerRegister        = TR::RealRegister::s11;
   _framePointerRegister        = TR::RealRegister::NoReg;

   _computedCallTargetRegister  = TR::RealRegister::t2;
   _vtableIndexArgumentRegister = TR::RealRegister::t3;
   _j9methodArgumentRegister    = TR::RealRegister::a0;

   _numberOfDependencyGPRegisters = 32; // To be determined
   _offsetToFirstLocal            = -8;

   initialize();
   }

const J9::RV::PrivateLinkageProperties
J9::RV::PrivateLinkage::_properties;

J9::RV::PrivateLinkage::PrivateLinkage(TR::CodeGenerator *cg)
   : J9::PrivateLinkage(cg),
   _interpretedMethodEntryPoint(NULL),
   _jittedMethodEntryPoint(NULL)
   {
   setOffsetToFirstParm(0); // To be determined
   }

const TR::RVLinkageProperties& J9::RV::PrivateLinkage::getProperties()
   {
   return _properties;
   }

uint32_t J9::RV::PrivateLinkage::getRightToLeft()
   {
   return getProperties().getRightToLeft();
   }

intptr_t
J9::RV::PrivateLinkage::entryPointFromCompiledMethod()
   {
   return reinterpret_cast<intptr_t>(getJittedMethodEntryPoint()->getBinaryEncoding());
   }

intptr_t
J9::RV::PrivateLinkage::entryPointFromInterpretedMethod()
   {
   return reinterpret_cast<intptr_t>(getInterpretedMethodEntryPoint()->getBinaryEncoding());
   }

void J9::RV::PrivateLinkage::mapStack(TR::ResolvedMethodSymbol *method)
   {
   const TR::RVLinkageProperties& linkageProperties = getProperties();
   int32_t firstLocalOffset = linkageProperties.getOffsetToFirstLocal();
   uint32_t stackIndex = firstLocalOffset;
   int32_t lowGCOffset = stackIndex;

   TR::GCStackAtlas *atlas = cg()->getStackAtlas();

   // Map all garbage collected references together so can concisely represent
   // stack maps. They must be mapped so that the GC map index in each local
   // symbol is honoured.
   //
   uint32_t numberOfLocalSlotsMapped = atlas->getNumberOfSlotsMapped() - atlas->getNumberOfParmSlotsMapped();

   stackIndex -= numberOfLocalSlotsMapped * TR::Compiler->om.sizeofReferenceAddress();

   if (comp()->useCompressedPointers())
      {
      // If there are any local objects we have to make sure they are aligned properly
      // when compressed pointers are used.  Otherwise, pointer compression may clobber
      // part of the pointer.
      //
      // Each auto's GC index will have already been aligned, so just the starting stack
      // offset needs to be aligned.
      //
      uint32_t unalignedStackIndex = stackIndex;
      stackIndex &= ~(TR::Compiler->om.objectAlignmentInBytes() - 1);
      uint32_t paddingBytes = unalignedStackIndex - stackIndex;
      if (paddingBytes > 0)
         {
         TR_ASSERT((paddingBytes & (TR::Compiler->om.sizeofReferenceAddress() - 1)) == 0, "Padding bytes should be a multiple of the slot/pointer size");
         uint32_t paddingSlots = paddingBytes / TR::Compiler->om.sizeofReferenceAddress();
         atlas->setNumberOfSlotsMapped(atlas->getNumberOfSlotsMapped() + paddingSlots);
         }
      }

   ListIterator<TR::AutomaticSymbol> automaticIterator(&method->getAutomaticList());
   TR::AutomaticSymbol *localCursor;
   int32_t firstLocalGCIndex = atlas->getNumberOfParmSlotsMapped();

   // Map local references to set the stack position correct according to the GC map index
   //
   for (localCursor = automaticIterator.getFirst(); localCursor; localCursor = automaticIterator.getNext())
      {
      if (localCursor->getGCMapIndex() >= 0)
         {
         localCursor->setOffset(stackIndex + TR::Compiler->om.sizeofReferenceAddress() * (localCursor->getGCMapIndex() - firstLocalGCIndex));
         if (localCursor->getGCMapIndex() == atlas->getIndexOfFirstInternalPointer())
            {
            atlas->setOffsetOfFirstInternalPointer(localCursor->getOffset() - firstLocalOffset);
            }
         }
      }

   method->setObjectTempSlots((lowGCOffset - stackIndex) / TR::Compiler->om.sizeofReferenceAddress());
   lowGCOffset = stackIndex;

   // Now map the rest of the locals
   //
   automaticIterator.reset();
   localCursor = automaticIterator.getFirst();

   while (localCursor != NULL)
      {
      if (localCursor->getGCMapIndex() < 0 &&
          localCursor->getSize() != 8)
         {
         mapSingleAutomatic(localCursor, stackIndex);
         }

      localCursor = automaticIterator.getNext();
      }

   automaticIterator.reset();
   localCursor = automaticIterator.getFirst();

   while (localCursor != NULL)
      {
      if (localCursor->getGCMapIndex() < 0 &&
          localCursor->getSize() == 8)
         {
         stackIndex -= (stackIndex & 0x4)?4:0;
         mapSingleAutomatic(localCursor, stackIndex);
         }

      localCursor = automaticIterator.getNext();
      }

   method->setLocalMappingCursor(stackIndex);

   // Map the parameters
   //
   ListIterator<TR::ParameterSymbol> parameterIterator(&method->getParameterList());
   TR::ParameterSymbol *parmCursor = parameterIterator.getFirst();

   int32_t offsetToFirstParm = getOffsetToFirstParm();
   uint32_t sizeOfParameterArea = method->getNumParameterSlots() * TR::Compiler->om.sizeofReferenceAddress();

   while (parmCursor != NULL)
      {
      uint32_t parmSize = (parmCursor->getDataType() != TR::Address) ? parmCursor->getSize()*2 : parmCursor->getSize();

      parmCursor->setParameterOffset(sizeOfParameterArea -
                                     parmCursor->getParameterOffset() -
                                     parmSize +
                                     offsetToFirstParm);

      parmCursor = parameterIterator.getNext();
      }

   atlas->setLocalBaseOffset(lowGCOffset - firstLocalOffset);
   atlas->setParmBaseOffset(atlas->getParmBaseOffset() + offsetToFirstParm - firstLocalOffset);
   }

void J9::RV::PrivateLinkage::mapSingleAutomatic(TR::AutomaticSymbol *p, uint32_t &stackIndex)
   {
   int32_t roundup = (comp()->useCompressedPointers() && p->isLocalObject() ? TR::Compiler->om.objectAlignmentInBytes() : TR::Compiler->om.sizeofReferenceAddress()) - 1;
   int32_t roundedSize = (p->getSize() + roundup) & (~roundup);
   if (roundedSize == 0)
      roundedSize = 4;

   p->setOffset(stackIndex -= roundedSize);
   }

void J9::RV::PrivateLinkage::initRVRealRegisterLinkage()
   {
   TR::Machine *machine = cg()->machine();

   FOR_EACH_RESERVED_REGISTER(machine, _properties,
         reg->setState(TR::RealRegister::Locked);
         reg->setAssignedRegister(reg);
   );

   FOR_EACH_REGISTER(machine, reg->setWeight(0xf000));

   // prefer preserved registers over the rest since they're saved / restored
   // in prologue/epilogue.
   FOR_EACH_CALLEE_SAVED_REGISTER(machine, _properties, reg->setWeight(0x0001));
   }

void
J9::RV::PrivateLinkage::setParameterLinkageRegisterIndex(TR::ResolvedMethodSymbol *method)
   {
   ListIterator<TR::ParameterSymbol> paramIterator(&(method->getParameterList()));
   TR::ParameterSymbol *paramCursor = paramIterator.getFirst();
   int32_t numIntArgs = 0, numFloatArgs = 0;
   const TR::RVLinkageProperties& properties = getProperties();

   while ( (paramCursor!=NULL) &&
           ( (numIntArgs < properties.getNumIntArgRegs()) ||
             (numFloatArgs < properties.getNumFloatArgRegs()) ) )
      {
      int32_t index = -1;

      switch (paramCursor->getDataType())
         {
         case TR::Int8:
         case TR::Int16:
         case TR::Int32:
         case TR::Int64:
         case TR::Address:
            if (numIntArgs < properties.getNumIntArgRegs())
               {
               index = numIntArgs;
               }
            numIntArgs++;
            break;

         case TR::Float:
         case TR::Double:
            if (numFloatArgs < properties.getNumFloatArgRegs())
               {
               index = numFloatArgs;
               }
            numFloatArgs++;
            break;
         }

      paramCursor->setLinkageRegisterIndex(index);
      paramCursor = paramIterator.getNext();
      }
   }


int32_t
J9::RV::PrivateLinkage::calculatePreservedRegisterSaveSize(
      uint32_t &registerSaveDescription,
      uint32_t &numGPRsSaved)
   {
   TR::Machine *machine = cg()->machine();

   FOR_EACH_CALLEE_SAVED_REGISTER(machine, _properties, {
      if (reg->getHasBeenAssignedInMethod())
         {
         registerSaveDescription |= 1 << (reg->getRegisterNumber() - 1);
         numGPRsSaved++;
         }
   });

   return numGPRsSaved*8;
   }


void J9::RV::PrivateLinkage::createPrologue(TR::Instruction *cursor)
   {

   // Prologues are emitted post-RA so it is fine to use real registers directly
   // in instructions
   //
   const TR::RVLinkageProperties& properties = getProperties();
   TR::Machine *machine = cg()->machine();
   TR::RealRegister *zero = machine->getRealRegister(TR::RealRegister::zero);
   TR::RealRegister *vmThread = machine->getRealRegister(properties.getVMThreadRegister());   // s10
   TR::RealRegister *javaSP = machine->getRealRegister(properties.getStackPointerRegister()); // s11

   TR::Instruction *beforeInterpreterMethodEntryPointInstruction = cursor;

   // --------------------------------------------------------------------------
   // Create the entry point when transitioning from an interpreted method.
   // Parameters are passed on the stack, so load them into the appropriate
   // linkage registers expected by the JITed method entry point.
   //
   cursor = loadStackParametersToLinkageRegisters(cursor);

   TR::Instruction *beforeJittedMethodEntryPointInstruction = cursor;

   // Entry breakpoint
   //
   if (comp()->getOption(TR_EntryBreakPoints))
      {
      cursor = generateITYPE(TR::InstOpCode::_ebreak, NULL, zero, zero, 0, cg(), cursor);
      }

   // --------------------------------------------------------------------------
   // Determine the bitvector of registers to preserve in the prologue
   //
   uint32_t registerSaveDescription = 0;
   uint32_t numGPRsSaved = 0;

   uint32_t preservedRegisterSaveSize = calculatePreservedRegisterSaveSize(registerSaveDescription, numGPRsSaved);

   // Offset between the entry JavaSP of a method and the first mapped local.  This covers
   // the space needed to preserve the RA.  It is a negative (or zero) offset.
   //
   int32_t firstLocalOffset = properties.getOffsetToFirstLocal();

   // The localMappingCursor is a negative-offset mapping of locals (autos and spills) to
   // the stack relative to the entry JavaSP of a method.  It includes the offset to the
   // first mapped local.
   //
   TR::ResolvedMethodSymbol *bodySymbol = comp()->getJittedMethodSymbol();
   int32_t localsSize = -(int32_t)(bodySymbol->getLocalMappingCursor());

   // Size of the frame needed to handle the argument storage requirements of any method
   // call in the current method.
   //
   // The offset to the first parm is the offset between the entry JavaSP and the first
   // mapped parameter.  It is a positive (or zero) offset.
   //
   int32_t outgoingArgsSize = cg()->getLargestOutgoingArgSize() + getOffsetToFirstParm();

   int32_t frameSizeIncludingReturnAddress = preservedRegisterSaveSize + localsSize + outgoingArgsSize;

   // Align the frame to 16 bytes
   //
   int32_t alignedFrameSizeIncludingReturnAddress = (frameSizeIncludingReturnAddress + 15) & ~15;

   // The frame size maintained by the code generator does not include the RA
   //
   cg()->setFrameSizeInBytes(alignedFrameSizeIncludingReturnAddress + firstLocalOffset);

   // --------------------------------------------------------------------------
   // Encode register save description (RSD)
   //
   int32_t preservedRegisterOffsetFromJavaBP = (alignedFrameSizeIncludingReturnAddress - outgoingArgsSize + firstLocalOffset);

   TR_ASSERT_FATAL(preservedRegisterOffsetFromJavaBP >= 0, "expecting a positive preserved register area offset");

   // Frame size is too large for the RSD word in the metadata
   //
   if (preservedRegisterOffsetFromJavaBP > 0xffff)
      {
      comp()->failCompilation<TR::CompilationInterrupted>("Overflowed or underflowed bounds of regSaveOffset in calculateFrameSize.");
      }

   registerSaveDescription |= (preservedRegisterOffsetFromJavaBP & 0xffff);

   cg()->setRegisterSaveDescription(registerSaveDescription);

   // In FSD, we must save linkage regs to the incoming argument area because
   // the stack overflow check doesn't preserve them.
   bool parmsHaveBeenStored = false;
   if (comp()->getOption(TR_FullSpeedDebug))
      {
      cursor = saveParametersToStack(cursor);
      parmsHaveBeenStored = true;
      }

   // --------------------------------------------------------------------------
   // Store return address (RA)
   //
   TR::MemoryReference *returnAddressMR = new (cg()->trHeapMemory()) TR::MemoryReference(javaSP, firstLocalOffset, cg());
   cursor = generateSTORE(TR::InstOpCode::_sd, NULL, returnAddressMR, machine->getRealRegister(TR::RealRegister::ra), cg(), cursor);

   // --------------------------------------------------------------------------
   // Speculatively adjust Java SP with the needed frame size.
   // This includes the preserved RA slot.
   //
   if (VALID_ITYPE_IMM(alignedFrameSizeIncludingReturnAddress))
      {
      cursor = generateITYPE(TR::InstOpCode::_addi, NULL, javaSP, javaSP, -alignedFrameSizeIncludingReturnAddress, cg(), cursor);
      }
   else
      {
      TR::Register *tmpReg = cg()->getTempRegister();
      cursor = loadConstant32(cg(), NULL, alignedFrameSizeIncludingReturnAddress, tmpReg, cursor);
      cursor = generateRTYPE(TR::InstOpCode::_sub, NULL, javaSP, javaSP, tmpReg, cg(), cursor);
      }

   // --------------------------------------------------------------------------
   // Perform javaSP overflow check
   //
   if (!comp()->isDLT())
      {
      //    if (javaSP < vmThread->SOM)
      //       goto stackOverflowSnippetLabel
      //
      // stackOverflowRestartLabel:
      //
      TR::MemoryReference *somMR = new (cg()->trHeapMemory()) TR::MemoryReference(vmThread, cg()->getStackLimitOffset(), cg());
      TR::RealRegister *somReg = machine->getRealRegister(TR::RealRegister::RegNum::t3);
      cursor = generateLOAD(TR::InstOpCode::_ld, NULL, somReg, somMR, cg(), cursor);

      TR::LabelSymbol *stackOverflowSnippetLabel = generateLabelSymbol(cg());
      cursor = generateBTYPE(TR::InstOpCode::_blt, NULL, stackOverflowSnippetLabel, javaSP, somReg, cg(), cursor);

      TR::LabelSymbol *stackOverflowRestartLabel = generateLabelSymbol(cg());
      cursor = generateLABEL(cg(), TR::InstOpCode::label, NULL, stackOverflowRestartLabel, cursor);

      cg()->addSnippet(new (cg()->trHeapMemory()) TR::RVStackCheckFailureSnippet(cg(), NULL, stackOverflowRestartLabel, stackOverflowSnippetLabel));
      }

   // --------------------------------------------------------------------------
   // Preserve GPRs
   //
   // javaSP has been adjusted, so preservedRegs start at offset outgoingArgSize
   // relative to the javaSP
   //
   // Registers are preserved in order from low reg number to hight reg number
   //
   if (numGPRsSaved)
      {
      int32_t offset = outgoingArgsSize;

      FOR_EACH_ASSIGNED_CALLEE_SAVED_REGISTER(machine, _properties, {
            TR::MemoryReference *stackSlot = new (cg()->trHeapMemory()) TR::MemoryReference(javaSP, offset, cg());
            cursor = generateSTORE(TR::InstOpCode::_sd, NULL, stackSlot, reg, cg(), cursor);
            offset += 8;
            numGPRsSaved--;
      });

      TR_ASSERT_FATAL(numGPRsSaved == 0, "preserved register mismatch in prologue");
      }

   // --------------------------------------------------------------------------
   // Initialize locals
   //
   TR::GCStackAtlas *atlas = cg()->getStackAtlas();
   if (atlas)
      {
      // The GC stack maps are conservative in that they all say that
      // collectable locals are live. This means that these locals must be
      // cleared out in case a GC happens before they are allocated a valid
      // value.
      // The atlas contains the number of locals that need to be cleared. They
      // are all mapped together starting at GC index 0.
      //
      uint32_t numLocalsToBeInitialized = atlas->getNumberOfSlotsToBeInitialized();
      if (numLocalsToBeInitialized > 0 || atlas->getInternalPointerMap())
         {
         // The LocalBaseOffset and firstLocalOffset are either negative or zero values
         //
         int32_t initializedLocalsOffsetFromAdjustedJavaSP = alignedFrameSizeIncludingReturnAddress + atlas->getLocalBaseOffset() + firstLocalOffset;

         for (int32_t i = 0; i < numLocalsToBeInitialized; i++, initializedLocalsOffsetFromAdjustedJavaSP += (TR::Compiler->om.sizeofReferenceAddress() * 2))
            {
            TR::MemoryReference *localMR = new (cg()->trHeapMemory()) TR::MemoryReference(javaSP, initializedLocalsOffsetFromAdjustedJavaSP, cg());
            cursor = generateSTORE(TR::InstOpCode::_sd, NULL, localMR, zero, cg(), cursor);
            }

         if (atlas->getInternalPointerMap())
            {
            TR_ASSERT_FATAL(0, "internal pointer initialization not available yet");
            }
         }
      }

   // Adjust final offsets on locals and parm symbols now that the frame size is known.
   // These offsets are relative to the javaSP which has been adjusted downward to
   // accommodate the frame of this method.
   //
   ListIterator<TR::AutomaticSymbol> automaticIterator(&bodySymbol->getAutomaticList());
   TR::AutomaticSymbol *localCursor = automaticIterator.getFirst();

   while (localCursor != NULL)
      {
      localCursor->setOffset(localCursor->getOffset() + alignedFrameSizeIncludingReturnAddress);
      localCursor = automaticIterator.getNext();
      }

   ListIterator<TR::ParameterSymbol> parameterIterator(&bodySymbol->getParameterList());
   TR::ParameterSymbol *parmCursor = parameterIterator.getFirst();
   while (parmCursor != NULL)
      {
      parmCursor->setParameterOffset(parmCursor->getParameterOffset() + alignedFrameSizeIncludingReturnAddress);
      parmCursor = parameterIterator.getNext();
      }

   // Ensure arguments reside where the method body expects them to be (either in registers or
   // on the stack).  This state is influenced by global register assignment.
   //
   cursor = copyParametersToHomeLocation(cursor, parmsHaveBeenStored);

   // Set the instructions for method entry points
   setInterpretedMethodEntryPoint(beforeInterpreterMethodEntryPointInstruction->getNext());
   setJittedMethodEntryPoint(beforeJittedMethodEntryPointInstruction->getNext());
   }

void J9::RV::PrivateLinkage::createEpilogue(TR::Instruction *cursor)
   {
   const TR::RVLinkageProperties& properties = getProperties();
   TR::Machine *machine = cg()->machine();
   TR::Node *lastNode = cursor->getNode();
   TR::ResolvedMethodSymbol *bodySymbol = comp()->getJittedMethodSymbol();
   TR::RealRegister *javaSP = machine->getRealRegister(properties.getStackPointerRegister()); // x20

   // restore preserved GPRs
   int32_t preservedRegisterOffsetFromJavaSP = cg()->getLargestOutgoingArgSize() + getOffsetToFirstParm(); // outgoingArgsSize
   FOR_EACH_ASSIGNED_CALLEE_SAVED_REGISTER(machine, _properties,
         {
         TR::MemoryReference *stackSlot = new (trHeapMemory()) TR::MemoryReference(javaSP, preservedRegisterOffsetFromJavaSP, cg());
         cursor = generateLOAD(TR::InstOpCode::_ld, lastNode, reg, stackSlot, cg(), cursor);
         preservedRegisterOffsetFromJavaSP += 8;
         });

   // remove space for preserved registers
   int32_t firstLocalOffset = properties.getOffsetToFirstLocal();

   uint32_t alignedFrameSizeIncludingReturnAddress = cg()->getFrameSizeInBytes() - firstLocalOffset;
   if (VALID_ITYPE_IMM(alignedFrameSizeIncludingReturnAddress))
      {
      cursor = generateITYPE(TR::InstOpCode::_addi, lastNode, javaSP, javaSP, alignedFrameSizeIncludingReturnAddress, cg(), cursor);
      }
   else
      {
      TR::Register *tmpReg = cg()->getTempRegister();
      cursor = loadConstant32(cg(), lastNode, alignedFrameSizeIncludingReturnAddress, tmpReg, cursor);
      cursor = generateRTYPE(TR::InstOpCode::_add, lastNode, javaSP, javaSP, tmpReg, cg(), cursor);
      }

   // restore return address
   TR::RealRegister *ra = machine->getRealRegister(TR::RealRegister::ra);
   if (machine->getLinkRegisterKilled())
      {
      TR::MemoryReference *returnAddressMR = new (cg()->trHeapMemory()) TR::MemoryReference(javaSP, firstLocalOffset, cg());
      cursor = generateLOAD(TR::InstOpCode::_ld, lastNode, ra, returnAddressMR, cg(), cursor);
      }

   TR::RealRegister *zero = machine->getRealRegister(TR::RealRegister::zero);
   cursor = generateITYPE(TR::InstOpCode::_jalr, lastNode, zero, ra, 0, cg(), cursor);
   }

void J9::RV::PrivateLinkage::pushOutgoingMemArgument(TR::Register *argReg, int32_t offset, TR::InstOpCode::Mnemonic opCode, TR::RVMemoryArgument &memArg)
   {
   const TR::RVLinkageProperties& properties = self()->getProperties();
   TR::RealRegister *javaSP = cg()->machine()->getRealRegister(properties.getStackPointerRegister()); // x20

   TR::MemoryReference *result = new (self()->trHeapMemory()) TR::MemoryReference(javaSP, offset, cg());
   memArg.argRegister = argReg;
   memArg.argMemory = result;
   memArg.opCode = opCode;
   }

int32_t J9::RV::PrivateLinkage::buildArgs(TR::Node *callNode,
   TR::RegisterDependencyConditions *dependencies)
   {
   return buildPrivateLinkageArgs(callNode, dependencies, TR_Private);
   }

int32_t J9::RV::PrivateLinkage::buildPrivateLinkageArgs(TR::Node *callNode,
   TR::RegisterDependencyConditions *dependencies,
   TR_LinkageConventions linkage)
   {
   TR_ASSERT(linkage == TR_Private || linkage == TR_Helper || linkage == TR_CHelper, "Unexpected linkage convention");

   const TR::RVLinkageProperties& properties = getProperties();
   TR::RVMemoryArgument *pushToMemory = NULL;
   TR::Register *tempReg;
   int32_t argIndex = 0;
   int32_t numMemArgs = 0;
   int32_t memArgSize = 0;
   int32_t firstExplicitArg = 0;
   int32_t from, to, step;
   int32_t argSize = -getOffsetToFirstParm();
   int32_t totalSize = 0;
   int32_t multiplier;

   uint32_t numIntegerArgs = 0;
   uint32_t numFloatArgs = 0;

   TR::Node *child;
   TR::DataType childType;
   TR::DataType resType = callNode->getType();

   uint32_t firstArgumentChild = callNode->getFirstArgumentIndex();

   TR::MethodSymbol *callSymbol = callNode->getSymbol()->castToMethodSymbol();

   bool isHelperCall = linkage == TR_Helper || linkage == TR_CHelper;
   bool rightToLeft = isHelperCall &&
                      //we want the arguments for induceOSR to be passed from left to right as in any other non-helper call
                      !callNode->getSymbolReference()->isOSRInductionHelper();

   if (rightToLeft)
      {
      from = callNode->getNumChildren() - 1;
      to   = firstArgumentChild;
      step = -1;
      }
   else
      {
      from = firstArgumentChild;
      to   = callNode->getNumChildren() - 1;
      step = 1;
      }

   uint32_t numIntArgRegs = properties.getNumIntArgRegs();
   uint32_t numFloatArgRegs = properties.getNumFloatArgRegs();

   TR::RealRegister::RegNum specialArgReg = TR::RealRegister::NoReg;
   switch (callSymbol->getMandatoryRecognizedMethod())
      {
      // Node: special long args are still only passed in one GPR
      case TR::java_lang_invoke_ComputedCalls_dispatchJ9Method:
         specialArgReg = getProperties().getJ9MethodArgumentRegister();
         // Other args go in memory
         numIntArgRegs   = 0;
         numFloatArgRegs = 0;
         break;
      case TR::java_lang_invoke_ComputedCalls_dispatchVirtual:
         specialArgReg = getProperties().getVTableIndexArgumentRegister();
         break;
#if 0
      case TR::java_lang_invoke_MethodHandle_invokeWithArgumentsHelper:
         numIntArgRegs   = 0;
         numFloatArgRegs = 0;
         break;
#endif
      }
   if (specialArgReg != TR::RealRegister::NoReg)
      {
      if (comp()->getOption(TR_TraceCG))
         {
         traceMsg(comp(), "Special arg %s in %s\n",
            comp()->getDebug()->getName(callNode->getChild(from)),
            comp()->getDebug()->getName(cg()->machine()->getRealRegister(specialArgReg)));
         }
      // Skip the special arg in the first loop
      from += step;
      }

   // C helpers have an implicit first argument (the VM thread) that we have to account for
   if (linkage == TR_CHelper)
      {
      TR_ASSERT(numIntArgRegs > 0, "This code doesn't handle passing this implicit arg on the stack");
      numIntegerArgs++;
      totalSize += TR::Compiler->om.sizeofReferenceAddress();
      }

   for (int32_t i = from; (rightToLeft && i >= to) || (!rightToLeft && i <= to); i += step)
      {
      child = callNode->getChild(i);
      childType = child->getDataType();

      switch (childType)
         {
         case TR::Int8:
         case TR::Int16:
         case TR::Int32:
         case TR::Int64:
         case TR::Address:
            multiplier = (childType == TR::Int64) ? 2 : 1;
            if (numIntegerArgs >= numIntArgRegs)
               {
               numMemArgs++;
               memArgSize += TR::Compiler->om.sizeofReferenceAddress() * multiplier;
               }
            numIntegerArgs++;
            totalSize += TR::Compiler->om.sizeofReferenceAddress() * multiplier;
            break;
         case TR::Float:
         case TR::Double:
            multiplier = (childType == TR::Double) ? 2 : 1;
            if (numFloatArgs >= numFloatArgRegs)
               {
               numMemArgs++;
               memArgSize += TR::Compiler->om.sizeofReferenceAddress() * multiplier;
               }
            numFloatArgs++;
            totalSize += TR::Compiler->om.sizeofReferenceAddress() * multiplier;
            break;
         default:
            TR_ASSERT(false, "Argument type %s is not supported\n", childType.toString());
         }
      }

   // From here, down, any new stack allocations will expire / die when the function returns
   TR::StackMemoryRegion stackMemoryRegion(*trMemory());

   if (numMemArgs > 0)
      {
      pushToMemory = new (trStackMemory()) TR::RVMemoryArgument[numMemArgs];
      }

   if (specialArgReg)
      from -= step;  // we do want to process special args in the following loop

   numIntegerArgs = 0;
   numFloatArgs = 0;

   // C helpers have an implicit first argument (the VM thread) that we have to account for
   if (linkage == TR_CHelper)
      {
      TR_ASSERT(numIntArgRegs > 0, "This code doesn't handle passing this implicit arg on the stack");
      TR::Register *vmThreadArgRegister = cg()->allocateRegister();
      generateITYPE(TR::InstOpCode::_addi, callNode, vmThreadArgRegister, cg()->getMethodMetaDataRegister(), 0, cg());
      dependencies->addPreCondition(vmThreadArgRegister, properties.getIntegerArgumentRegister(numIntegerArgs));
      if (resType.getDataType() == TR::NoType)
         dependencies->addPostCondition(vmThreadArgRegister, properties.getIntegerArgumentRegister(numIntegerArgs));
      numIntegerArgs++;
      firstExplicitArg = 1;
      }

   // Helper linkage preserves all argument registers except the return register
   // TODO: C helper linkage does not, this code needs to make sure argument registers are killed in post dependencies
   for (int32_t i = from; (rightToLeft && i >= to) || (!rightToLeft && i <= to); i += step)
      {
      TR::Register *argRegister;
      TR::InstOpCode::Mnemonic op;
      bool isSpecialArg = (i == from && specialArgReg != TR::RealRegister::NoReg);

      child = callNode->getChild(i);
      childType = child->getDataType();

      switch (childType)
         {
         case TR::Int8:
         case TR::Int16:
         case TR::Int32:
         case TR::Int64:
         case TR::Address:
            if (childType == TR::Address)
               {
               argRegister = pushAddressArg(child);
               }
            else if (childType == TR::Int64)
               {
               argRegister = pushLongArg(child);
               }
            else
               {
               argRegister = pushIntegerWordArg(child);
               }
            if (isSpecialArg)
               {
               if (specialArgReg == properties.getIntegerReturnRegister(0))
                  {
                  TR::Register *resultReg;
                  if (resType.isAddress())
                     resultReg = cg()->allocateCollectedReferenceRegister();
                  else
                     resultReg = cg()->allocateRegister();
                  dependencies->addPreCondition(argRegister, specialArgReg);
                  dependencies->addPostCondition(resultReg, properties.getIntegerReturnRegister(0));
                  }
               else
                  {
                  TR::addDependency(dependencies, argRegister, specialArgReg, TR_GPR, cg());
                  }
               }
            else
               {
               argSize += TR::Compiler->om.sizeofReferenceAddress() * ((childType == TR::Int64) ? 2 : 1);
               if (numIntegerArgs < numIntArgRegs)
                  {
                  if (!cg()->canClobberNodesRegister(child, 0))
                     {
                     if (argRegister->containsCollectedReference())
                        tempReg = cg()->allocateCollectedReferenceRegister();
                     else
                        tempReg = cg()->allocateRegister();
                     generateITYPE(TR::InstOpCode::_addi, callNode, tempReg, argRegister, 0, cg());
                     argRegister = tempReg;
                     }
                  if (numIntegerArgs == firstExplicitArg)
                     {
                     // the first integer argument
                     TR::Register *resultReg;
                     if (resType.isAddress())
                        resultReg = cg()->allocateCollectedReferenceRegister();
                     else
                        resultReg = cg()->allocateRegister();
                     dependencies->addPreCondition(argRegister, properties.getIntegerArgumentRegister(numIntegerArgs));
                     dependencies->addPostCondition(resultReg, TR::RealRegister::x0);
                     if (firstExplicitArg == 1)
                        dependencies->addPostCondition(argRegister, properties.getIntegerArgumentRegister(numIntegerArgs));
                     }
                  else
                     {
                     TR::addDependency(dependencies, argRegister, properties.getIntegerArgumentRegister(numIntegerArgs), TR_GPR, cg());
                     }
                  }
               else // numIntegerArgs >= numIntArgRegs
                  {
                  op = ((childType == TR::Address) || (childType == TR::Int64)) ? TR::InstOpCode::_sd : TR::InstOpCode::_sw;
                  pushOutgoingMemArgument(argRegister, totalSize - argSize, op, pushToMemory[argIndex++]);
                  }
               numIntegerArgs++;
               }
            break;
         case TR::Float:
         case TR::Double:
            if (childType == TR::Float)
               {
               argSize += TR::Compiler->om.sizeofReferenceAddress();
               argRegister = pushFloatArg(child);
               }
            else
               {
               argSize += TR::Compiler->om.sizeofReferenceAddress() * 2;
               argRegister = pushDoubleArg(child);
               }
            if (numFloatArgs < numFloatArgRegs)
               {
               if (!cg()->canClobberNodesRegister(child, 0))
                  {
                  tempReg = cg()->allocateRegister(TR_FPR);
                  op = (childType == TR::Float) ? TR::InstOpCode::_fsgnj_s : TR::InstOpCode::_fsgnj_d;
                  generateRTYPE(op, callNode, tempReg, argRegister, argRegister, cg());
                  argRegister = tempReg;
                  }
               if (numFloatArgs == 0 && resType.isFloatingPoint())
                  {
                  TR::Register *resultReg;
                  if (resType.getDataType() == TR::Float)
                     resultReg = cg()->allocateSinglePrecisionRegister();
                  else
                     resultReg = cg()->allocateRegister(TR_FPR);
                  dependencies->addPreCondition(argRegister, TR::RealRegister::fa0);
                  dependencies->addPostCondition(resultReg, TR::RealRegister::fa0);
                  }
               else
                  TR::addDependency(dependencies, argRegister, properties.getFloatArgumentRegister(numFloatArgs), TR_FPR, cg());
               }
            else // numFloatArgs >= numFloatArgRegs
               {
               op = (childType == TR::Float) ? TR::InstOpCode::_fsw : TR::InstOpCode::_fsd;
               pushOutgoingMemArgument(argRegister, totalSize - argSize, op, pushToMemory[argIndex++]);
               }
            numFloatArgs++;
            break;
         }
      }

   for (int32_t i = TR::RealRegister::FirstGPR; i <= TR::RealRegister::LastGPR; ++i)
      {
      TR::RealRegister::RegNum realReg = (TR::RealRegister::RegNum)i;
      if (properties.getPreserved(realReg) || (properties.getRegisterFlags(realReg) & RV_Reserved))
         continue;
      if (realReg == specialArgReg)
         continue; // already added deps above.  No need to add them here.
      if (callSymbol->isComputed() && i == getProperties().getComputedCallTargetRegister())
         continue;
      if (!dependencies->searchPreConditionRegister(realReg))
         {
         if (realReg == properties.getIntegerArgumentRegister(0) && callNode->getDataType() == TR::Address)
            {
            dependencies->addPreCondition(cg()->allocateRegister(), TR::RealRegister::x0);
            dependencies->addPostCondition(cg()->allocateCollectedReferenceRegister(), TR::RealRegister::x0);
            }
         else
            {
            // Helper linkage preserves all registers that are not argument registers, so we don't need to spill them.
            if (linkage != TR_Helper)
               TR::addDependency(dependencies, NULL, realReg, TR_GPR, cg());
            }
         }
      }

   if (callNode->getType().isFloatingPoint() && numFloatArgs == 0)
      {
      //add return floating-point register dependency
      TR::addDependency(dependencies, NULL, (TR::RealRegister::RegNum)getProperties().getFloatReturnRegister(), TR_FPR, cg());
      }

   for (int32_t i = TR::RealRegister::FirstFPR; i <= TR::RealRegister::LastFPR; ++i)
      {
      TR::RealRegister::RegNum realReg = (TR::RealRegister::RegNum)i;
      if (properties.getPreserved(realReg))
         continue;
      if (!dependencies->searchPreConditionRegister(realReg))
         {
         TR::addDependency(dependencies, NULL, realReg, TR_FPR, cg());
         }
      }

   if (numMemArgs > 0)
      {
      for (argIndex = 0; argIndex < numMemArgs; argIndex++)
         {
         TR::Register *aReg = pushToMemory[argIndex].argRegister;
         generateSTORE(pushToMemory[argIndex].opCode, callNode, pushToMemory[argIndex].argMemory, aReg, cg());
         cg()->stopUsingRegister(aReg);
         }
      }

   return totalSize;
   }

void J9::RV::PrivateLinkage::buildDirectCall(TR::Node *callNode,
   TR::SymbolReference *callSymRef,
   TR::RegisterDependencyConditions *dependencies,
   const TR::RVLinkageProperties &pp,
   uint32_t argSize)
   {
#if 0
   TR::Instruction *gcPoint;
   TR::MethodSymbol *callSymbol = callSymRef->getSymbol()->castToMethodSymbol();

   TR_J9VMBase *fej9 = (TR_J9VMBase *)(comp()->fe());

   if (callSymRef->getReferenceNumber() >= TR_RVnumRuntimeHelpers)
      fej9->reserveTrampolineIfNecessary(comp(), callSymRef, false);

   bool forceUnresolvedDispatch = fej9->forceUnresolvedDispatch();

   if (callSymbol->isJITInternalNative() ||
       (!callSymRef->isUnresolved() && !callSymbol->isInterpreted() &&
        ((forceUnresolvedDispatch && callSymbol->isHelper()) || !forceUnresolvedDispatch)))
      {
      bool isMyself = comp()->isRecursiveMethodTarget(callSymbol);

      gcPoint = generateImmSymInstruction(cg(), TR::InstOpCode::bl, callNode,
         isMyself ? 0 : (uintptr_t)callSymbol->getMethodAddress(),
         dependencies,
         callSymRef ? callSymRef : callNode->getSymbolReference(),
         NULL);
      }
   else
      {
      TR::LabelSymbol *label = generateLabelSymbol(cg());
      TR::Snippet *snippet;

      if (callSymRef->isUnresolved() || comp()->compileRelocatableCode())
         {
         snippet = new (trHeapMemory()) TR::RVUnresolvedCallSnippet(cg(), callNode, label, argSize);
         }
      else
         {
         snippet = new (trHeapMemory()) TR::RVCallSnippet(cg(), callNode, label, argSize);
         snippet->gcMap().setGCRegisterMask(pp.getPreservedRegisterMapForGC());
         }

      cg()->addSnippet(snippet);
      gcPoint = generateImmSymInstruction(cg(), TR::InstOpCode::bl, callNode,
         0, dependencies,
         new (trHeapMemory()) TR::SymbolReference(comp()->getSymRefTab(), label),
         snippet);

      // Nop is necessary due to confusion when resolving shared slots at a transition
      if (callSymRef->isOSRInductionHelper())
         cg()->generateNop(callNode);

      }

   gcPoint->RVNeedsGCMap(cg(), callSymbol->getLinkageConvention() == TR_Helper ? 0xffffffff : pp.getPreservedRegisterMapForGC());
#endif
   }

TR::Register *J9::RV::PrivateLinkage::buildDirectDispatch(TR::Node *callNode)
   {
#if 0
   TR::SymbolReference *callSymRef = callNode->getSymbolReference();
   const TR::RVLinkageProperties &pp = getProperties();
   TR::RegisterDependencyConditions *dependencies =
      new (trHeapMemory()) TR::RegisterDependencyConditions(
         pp.getNumberOfDependencyGPRegisters(),
         pp.getNumberOfDependencyGPRegisters(), trMemory());

   int32_t argSize = buildArgs(callNode, dependencies);

   buildDirectCall(callNode, callSymRef, dependencies, pp, argSize);
   cg()->machine()->setLinkRegisterKilled(true);

   TR::Register *retReg;
   switch(callNode->getOpCodeValue())
      {
      case TR::icall:
         retReg = dependencies->searchPostConditionRegister(
                     pp.getIntegerReturnRegister());
         break;
      case TR::lcall:
      case TR::acall:
         retReg = dependencies->searchPostConditionRegister(
                     pp.getLongReturnRegister());
         break;
      case TR::fcall:
      case TR::dcall:
         retReg = dependencies->searchPostConditionRegister(
                     pp.getFloatReturnRegister());
         break;
      case TR::call:
         retReg = NULL;
         break;
      default:
         retReg = NULL;
         TR_ASSERT_FATAL(false, "Unsupported direct call Opcode.");
      }

   callNode->setRegister(retReg);

   dependencies->stopUsingDepRegs(cg(), retReg);
   return retReg;
#endif
   }

static TR::Register *evaluateUpToVftChild(TR::Node *callNode, TR::CodeGenerator *cg)
   {
   TR::Register *vftReg = NULL;
   if (callNode->getFirstArgumentIndex() == 1)
      {
      TR::Node *child = callNode->getFirstChild();
      vftReg = cg->evaluate(child);
      cg->decReferenceCount(child);
      }
   TR_ASSERT_FATAL(vftReg != NULL, "Failed to find vft child.");
   return vftReg;
   }

void J9::RV::PrivateLinkage::buildVirtualDispatch(TR::Node *callNode,
   TR::RegisterDependencyConditions *dependencies,
   uint32_t argSize)
   {
#if 0
   TR::Register *x0 = dependencies->searchPreConditionRegister(TR::RealRegister::x0);
   TR::Register *x9 = dependencies->searchPreConditionRegister(TR::RealRegister::x9);

   TR::SymbolReference *methodSymRef = callNode->getSymbolReference();
   TR::MethodSymbol *methodSymbol = methodSymRef->getSymbol()->castToMethodSymbol();
   TR::LabelSymbol *doneLabel = NULL;
   uint32_t regMapForGC = getProperties().getPreservedRegisterMapForGC();
   void *thunk = NULL;

   TR::Instruction *gcPoint;

   TR_J9VMBase *fej9 = (TR_J9VMBase *)(comp()->fe());

   // Computed calls
   //
   if (methodSymbol->isComputed())
      {
      TR::Register *vftReg = evaluateUpToVftChild(callNode, cg());
      TR::addDependency(dependencies, vftReg, getProperties().getComputedCallTargetRegister(), TR_GPR, cg());

      switch (methodSymbol->getMandatoryRecognizedMethod())
         {
         case TR::java_lang_invoke_ComputedCalls_dispatchVirtual:
            {
            // Need a j2i thunk for the method that will ultimately be dispatched by this handle call
            char *j2iSignature = fej9->getJ2IThunkSignatureForDispatchVirtual(methodSymbol->getMethod()->signatureChars(), methodSymbol->getMethod()->signatureLength(), comp());
            int32_t signatureLen = strlen(j2iSignature);
            thunk = fej9->getJ2IThunk(j2iSignature, signatureLen, comp());
            if (!thunk)
               {
               thunk = fej9->setJ2IThunk(j2iSignature, signatureLen,
                                         TR::RVCallSnippet::generateVIThunk(fej9->getEquivalentVirtualCallNodeForDispatchVirtual(callNode, comp()), argSize, cg()), comp());
               }
            }
         default:
            if (fej9->needsInvokeExactJ2IThunk(callNode, comp()))
               {
               comp()->getPersistentInfo()->getInvokeExactJ2IThunkTable()->addThunk(
                  TR::RVCallSnippet::generateInvokeExactJ2IThunk(callNode, argSize, cg(), methodSymbol->getMethod()->signatureChars()), fej9);
               }
            break;
         }

      TR::Instruction *gcPoint = generateRegBranchInstruction(cg(), TR::InstOpCode::blr, callNode, vftReg, dependencies);
      gcPoint->RVNeedsGCMap(cg(), regMapForGC);

      return;
      }

   // Virtual and interface calls
   //
   TR_ASSERT_FATAL(methodSymbol->isVirtual() || methodSymbol->isInterface(), "Unexpected method type");

   thunk = fej9->getJ2IThunk(methodSymbol->getMethod(), comp());
   if (!thunk)
      thunk = fej9->setJ2IThunk(methodSymbol->getMethod(), TR::RVCallSnippet::generateVIThunk(callNode, argSize, cg()), comp());

   if (methodSymbol->isVirtual())
      {
      TR::MemoryReference *tempMR;
      TR::Register *vftReg = evaluateUpToVftChild(callNode, cg());
      TR::addDependency(dependencies, vftReg, TR::RealRegister::NoReg, TR_GPR, cg());

      if (methodSymRef->isUnresolved() || comp()->compileRelocatableCode())
         {
         doneLabel = generateLabelSymbol(cg());

         TR::LabelSymbol *vcSnippetLabel = generateLabelSymbol(cg());
         TR::RVVirtualUnresolvedSnippet *vcSnippet =
            new (trHeapMemory())
            TR::RVVirtualUnresolvedSnippet(cg(), callNode, vcSnippetLabel, argSize, doneLabel, (uint8_t *)thunk);
         cg()->addSnippet(vcSnippet);


         // The following instructions are modified by _virtualUnresolvedHelper
         // in riscv/runtime/PicBuilder.spp to load the vTable index in x9

         // This `b` instruction is modified to movzx x9, lower 16bit of offset
         generateLabelInstruction(cg(), TR::InstOpCode::b, callNode, vcSnippetLabel);
         generateTrg1ImmInstruction(cg(), TR::InstOpCode::movkx, callNode, x9, TR::MOV_LSL16);
         generateTrg1Src1ImmInstruction(cg(), TR::InstOpCode::sbfmx, callNode, x9, x9, 0x1F); // sxtw x9, w9
         tempMR = new (trHeapMemory()) TR::MemoryReference(vftReg, x9, cg());
         generateTrg1MemInstruction(cg(), TR::InstOpCode::ldroffx, callNode, x9, tempMR);
         gcPoint = generateRegBranchInstruction(cg(), TR::InstOpCode::blr, callNode, x9);
         }
      else
         {
         int32_t offset = methodSymRef->getOffset();
         TR_ASSERT(offset < 0, "Unexpected positive offset for virtual call");

         // jitVTableIndex() in oti/JITInterface.hpp assumes the instruction sequence below
         if (offset >= -65536)
            {
            generateTrg1ImmInstruction(cg(), TR::InstOpCode::movnx, callNode, x9, ~offset & 0xFFFF);
            }
         else
            {
            generateTrg1ImmInstruction(cg(), TR::InstOpCode::movzx, callNode, x9, offset & 0xFFFF);
            generateTrg1ImmInstruction(cg(), TR::InstOpCode::movkx, callNode, x9,
                                       (((offset >> 16) & 0xFFFF) | TR::MOV_LSL16));
            generateTrg1Src1ImmInstruction(cg(), TR::InstOpCode::sbfmx, callNode, x9, x9, 0x1F); // sxtw x9, w9
            }
         tempMR = new (trHeapMemory()) TR::MemoryReference(vftReg, x9, cg());
         generateTrg1MemInstruction(cg(), TR::InstOpCode::ldroffx, callNode, x9, tempMR);
         gcPoint = generateRegBranchInstruction(cg(), TR::InstOpCode::blr, callNode, x9, dependencies);
         }
      }
   else
      {
      // interface calls
      // ToDo: Inline interface dispatch
      doneLabel = generateLabelSymbol(cg());

      /**
       * The vft child is not used by this interface dispatch, but its reference
       * count must be decremented as if it were.
       */
      cg()->recursivelyDecReferenceCount(callNode->getFirstChild());

      TR::LabelSymbol *ifcSnippetLabel = generateLabelSymbol(cg());
      TR::RVInterfaceCallSnippet *ifcSnippet =
         new (trHeapMemory())
         TR::RVInterfaceCallSnippet(cg(), callNode, ifcSnippetLabel, argSize, doneLabel, (uint8_t *)thunk);
      cg()->addSnippet(ifcSnippet);

      gcPoint = generateLabelInstruction(cg(), TR::InstOpCode::b, callNode, ifcSnippetLabel);
      }

   gcPoint->RVNeedsGCMap(cg(), regMapForGC);

   if (doneLabel)
      generateLabelInstruction(cg(), TR::InstOpCode::label, callNode, doneLabel, dependencies);

   return;
#endif
   }

TR::Register *J9::RV::PrivateLinkage::buildIndirectDispatch(TR::Node *callNode)
   {
#if 0
   const TR::RVLinkageProperties &pp = getProperties();
   TR::RealRegister *sp = cg()->machine()->getRealRegister(pp.getStackPointerRegister());

   TR::RegisterDependencyConditions *dependencies =
      new (trHeapMemory()) TR::RegisterDependencyConditions(
         pp.getNumberOfDependencyGPRegisters(),
         pp.getNumberOfDependencyGPRegisters(), trMemory());

   int32_t argSize = buildArgs(callNode, dependencies);

   buildVirtualDispatch(callNode, dependencies, argSize);
   cg()->machine()->setLinkRegisterKilled(true);

   TR::Register *retReg;
   switch(callNode->getOpCodeValue())
      {
      case TR::icalli:
         retReg = dependencies->searchPostConditionRegister(
                     pp.getIntegerReturnRegister());
         break;
      case TR::lcalli:
      case TR::acalli:
         retReg = dependencies->searchPostConditionRegister(
                     pp.getLongReturnRegister());
         break;
      case TR::fcalli:
      case TR::dcalli:
         retReg = dependencies->searchPostConditionRegister(
                     pp.getFloatReturnRegister());
         break;
      case TR::calli:
         retReg = NULL;
         break;
      default:
         retReg = NULL;
         TR_ASSERT_FATAL(false, "Unsupported indirect call Opcode.");
      }

   callNode->setRegister(retReg);

   dependencies->stopUsingDepRegs(cg(), retReg);
   return retReg;
#endif
   }

TR::Instruction *
J9::RV::PrivateLinkage::loadStackParametersToLinkageRegisters(TR::Instruction *cursor)
   {
   TR::Machine *machine = cg()->machine();
   const TR::RVLinkageProperties& properties = getProperties();
   TR::RealRegister *javaSP = machine->getRealRegister(properties.getStackPointerRegister());       // x20

   TR::ResolvedMethodSymbol *bodySymbol = comp()->getJittedMethodSymbol();
   ListIterator<TR::ParameterSymbol> parmIterator(&(bodySymbol->getParameterList()));
   TR::ParameterSymbol *parmCursor;

   // Copy from stack all parameters that belong in linkage regs
   //
   for (parmCursor = parmIterator.getFirst();
        parmCursor != NULL;
        parmCursor = parmIterator.getNext())
      {
      if (parmCursor->isParmPassedInRegister())
         {
         int8_t lri = parmCursor->getLinkageRegisterIndex();
         TR::RealRegister *linkageReg;
         TR::InstOpCode::Mnemonic op;
         TR::DataType dataType = parmCursor->getDataType();

         if (dataType == TR::Double || dataType == TR::Float)
            {
            linkageReg = machine->getRealRegister(properties.getFloatArgumentRegister(lri));
            op = (dataType == TR::Double) ? TR::InstOpCode::_fld : TR::InstOpCode::_flw;
            }
         else
            {
            linkageReg = machine->getRealRegister(properties.getIntegerArgumentRegister(lri));
            op = (dataType == TR::Int64 || dataType == TR::Address) ? TR::InstOpCode::_ld : TR::InstOpCode::_lw;
            }

         TR::MemoryReference *stackMR = new (cg()->trHeapMemory()) TR::MemoryReference(javaSP, parmCursor->getParameterOffset(), cg());
         cursor = generateLOAD(op, NULL, linkageReg, stackMR, cg(), cursor);
         }
      }

    return cursor;
    }

TR::Instruction *
J9::RV::PrivateLinkage::saveParametersToStack(TR::Instruction *cursor)
   {
   TR::Machine *machine = cg()->machine();
   const TR::RVLinkageProperties& properties = getProperties();
   TR::RealRegister *javaSP = machine->getRealRegister(properties.getStackPointerRegister());       // x20

   TR::ResolvedMethodSymbol *bodySymbol = comp()->getJittedMethodSymbol();
   ListIterator<TR::ParameterSymbol> parmIterator(&(bodySymbol->getParameterList()));
   TR::ParameterSymbol *parmCursor;

   // Store to stack all parameters passed in linkage registers
   //
   for (parmCursor = parmIterator.getFirst();
        parmCursor != NULL;
        parmCursor = parmIterator.getNext())
      {
      if (parmCursor->isParmPassedInRegister())
         {
         int8_t lri = parmCursor->getLinkageRegisterIndex();
         TR::RealRegister *linkageReg;
         TR::InstOpCode::Mnemonic op;

         if (parmCursor->getDataType() == TR::Double || parmCursor->getDataType() == TR::Float)
            {
            linkageReg = machine->getRealRegister(properties.getFloatArgumentRegister(lri));
            op = (parmCursor->getDataType() == TR::Double) ? TR::InstOpCode::_fsd : TR::InstOpCode::_fsw;
            }
         else
            {
            linkageReg = machine->getRealRegister(properties.getIntegerArgumentRegister(lri));
            op = TR::InstOpCode::_sd;
            }

         TR::MemoryReference *stackMR = new (cg()->trHeapMemory()) TR::MemoryReference(javaSP, parmCursor->getParameterOffset(), cg());
         cursor = generateSTORE(op, NULL, stackMR, linkageReg, cg(), cursor);
         }
      }

   return cursor;
   }

void J9::RV::PrivateLinkage::performPostBinaryEncoding()
   {
   // --------------------------------------------------------------------------
   // Encode the size of the interpreter entry area into the linkage info word
   //
   TR_ASSERT_FATAL(cg()->getReturnTypeInfoInstruction(),
                   "Expecting the return type info instruction to be created");

   auto linkageInfoWordInstruction = cg()->getReturnTypeInfoInstruction();
   uint32_t linkageInfoWord = linkageInfoWordInstruction->getSourceImmediate();

   intptr_t jittedMethodEntryAddress = reinterpret_cast<intptr_t>(getJittedMethodEntryPoint()->getBinaryEncoding());
   intptr_t interpretedMethodEntryAddress = reinterpret_cast<intptr_t>(getInterpretedMethodEntryPoint()->getBinaryEncoding());

   linkageInfoWord = (static_cast<uint32_t>(jittedMethodEntryAddress - interpretedMethodEntryAddress) << 16) | linkageInfoWord;
   linkageInfoWordInstruction->setSourceImmediate(linkageInfoWord);

   *(uint32_t *)(linkageInfoWordInstruction->getBinaryEncoding()) = linkageInfoWord;

   // Set recompilation info
   //
   TR::Recompilation *recomp = comp()->getRecompilationInfo();
   if (recomp != NULL && recomp->couldBeCompiledAgain())
      {
      J9::PrivateLinkage::LinkageInfo *lkInfo = J9::PrivateLinkage::LinkageInfo::get(cg()->getCodeStart());
      if (recomp->useSampling())
         lkInfo->setSamplingMethodBody();
      else
         lkInfo->setCountingMethodBody();
      }
   }

int32_t J9::RV::HelperLinkage::buildArgs(TR::Node *callNode,
   TR::RegisterDependencyConditions *dependencies)
   {
   return buildPrivateLinkageArgs(callNode, dependencies, _helperLinkage);
   }
