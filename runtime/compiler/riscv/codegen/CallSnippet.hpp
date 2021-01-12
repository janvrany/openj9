/*******************************************************************************
 * Copyright (c) 2020, 2020 IBM Corp. and others
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

#ifndef RVCALLSNIPPET_INCL
#define RVCALLSNIPPET_INCL

#include "codegen/Snippet.hpp"
#include "env/VMJ9.h"

namespace TR { class CodeGenerator; }
class TR_J2IThunk;

extern void arm64CodeSync(uint8_t *codePointer, uint32_t codeSize);

namespace TR {

class RVCallSnippet : public TR::Snippet
   {
   uint8_t *callRA;
   int32_t sizeOfArguments;

   public:

   RVCallSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s)
      : TR::Snippet(cg, c, lab, false), sizeOfArguments(s), callRA(0)
      {
      }

   virtual Kind getKind() { return IsCall; }

   virtual uint8_t *emitSnippetBody();

   virtual uint32_t getLength(int32_t estimatedSnippetStart);

   int32_t getSizeOfArguments()          {return sizeOfArguments;}
   int32_t setSizeOfArguments(int32_t s) {return (sizeOfArguments = s);}

   TR_RuntimeHelper getHelper();

   uint8_t *getCallRA() {return callRA;}
   uint8_t *setCallRA(uint8_t *ra) {return (callRA=ra);}

   static uint8_t *generateVIThunk(TR::Node *callNode, int32_t argSize, TR::CodeGenerator *cg);
   static TR_J2IThunk *generateInvokeExactJ2IThunk(TR::Node *callNode, int32_t argSize, TR::CodeGenerator *cg, char *signature);
   };

class RVUnresolvedCallSnippet : public TR::RVCallSnippet
   {

   public:

   RVUnresolvedCallSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s)
      : TR::RVCallSnippet(cg, c, lab, s)
      {
      }

   virtual Kind getKind() { return IsUnresolvedCall; }

   virtual uint8_t *emitSnippetBody();

   virtual uint32_t getLength(int32_t estimatedSnippetStart);
   };

class RVVirtualSnippet : public TR::Snippet
   {
   TR::LabelSymbol *returnLabel;
   int32_t sizeOfArguments;

   public:

   RVVirtualSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s, TR::LabelSymbol *retl)
      : TR::Snippet(cg, c, lab, true), sizeOfArguments(s), returnLabel(retl)
      {
      }

   virtual Kind getKind() { return IsVirtual; }

   virtual uint8_t *emitSnippetBody();

   virtual uint32_t getLength(int32_t estimatedSnippetStart);

   int32_t getSizeOfArguments()          {return sizeOfArguments;}
   int32_t setSizeOfArguments(int32_t s) {return (sizeOfArguments = s);}

   TR::LabelSymbol *getReturnLabel() {return returnLabel;}
   TR::LabelSymbol *setReturnLabel(TR::LabelSymbol *rl) {return (returnLabel=rl);}
   };

class RVVirtualUnresolvedSnippet : public TR::RVVirtualSnippet
   {
   uint8_t *thunkAddress;
   public:

   RVVirtualUnresolvedSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s, TR::LabelSymbol *retl)
      : TR::RVVirtualSnippet(cg, c, lab, s, retl), thunkAddress(NULL)
      {
      }

   RVVirtualUnresolvedSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s, TR::LabelSymbol *retl, uint8_t *thunkPtr)
      : TR::RVVirtualSnippet(cg, c, lab, s, retl), thunkAddress(thunkPtr)
      {
      }

   virtual Kind getKind() { return IsVirtualUnresolved; }

   virtual uint8_t *emitSnippetBody();

   virtual uint32_t getLength(int32_t estimatedSnippetStart);
   };

class RVInterfaceCallSnippet : public TR::RVVirtualSnippet
   {
   uint8_t *thunkAddress;

   public:

   RVInterfaceCallSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s, TR::LabelSymbol *retl)
      : TR::RVVirtualSnippet(cg, c, lab, s, retl), thunkAddress(NULL)
      {
      }

   RVInterfaceCallSnippet(TR::CodeGenerator *cg, TR::Node *c, TR::LabelSymbol *lab, int32_t s, TR::LabelSymbol *retl, uint8_t *thunkPtr)
      : TR::RVVirtualSnippet(cg, c, lab, s, retl), thunkAddress(thunkPtr)
      {
      }

   virtual Kind getKind() { return IsInterfaceCall; }

   virtual uint8_t *emitSnippetBody();

   virtual uint32_t getLength(int32_t estimatedSnippetStart);
   };

}

#endif
