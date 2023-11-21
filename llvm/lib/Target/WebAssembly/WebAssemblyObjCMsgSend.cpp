//===-- WebAssemblyObjCMsgSend.cpp -  --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
///
/// Refactor objc_msgSend calls into objc_msgSend_vii by their signature.
///
//===----------------------------------------------------------------------===//


#include "WebAssembly.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Analysis/ConstantFolding.h"
#include "llvm/Analysis/InstructionSimplify.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/IR/CFG.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Transforms/Utils/ModuleUtils.h"
#include <map>

#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "wasm_gen_objc_funcs"

// TODO:
// - CallSite is either removed or renamed to CallBase.
// - Ensure that linker generates all objc_msgSend for any Objective-C method defined in the lib rather than just
//   the ones that are called, otherwise it wont work with the dynamics of Objcive-C as runtime can be altered at runtime.

using namespace llvm;

namespace {

    const std::string OBJC_MSG_SEND("objc_msgSend");
    const std::string OBJC_MSG_SEND_ST("objc_msgSend_stret");
    const std::string OBJC_MSG_SEND_FP("objc_msgSend_fpret");
    const std::string OBJC_METHOD_INVOKE("method_invoke");

    enum class MsgSendMethodType {
        NONE,
        MSG_SEND,
        MSG_SEND_ST,
        MSG_SEND_FP,
        METHOD_INVOKE,
    };

class WebAssemblyGenObjCMsgSendFuncs final : public llvm::ModulePass {

    StringRef getPassName() const override {
        return "WebAssembly Fix objc_msgSend Bitcasts";
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
        AU.setPreservesCFG();
        ModulePass::getAnalysisUsage(AU);
    }

    bool runOnModule(Module &M) override;

    public:
        static char ID;
        WebAssemblyGenObjCMsgSendFuncs() : ModulePass(ID) {}
    };

    /*
    class ObjcFunction {
    public:
        ObjcFunction(std::string name, llvm::FunctionType *messageFunctionType, llvm::Module *module);
        ObjcFunction(const ObjcFunction& other) : m_name(other.m_name), m_messageFunctionType(other.m_messageFunctionType) {};
        ~ObjcFunction() {};

        bool operator<(const ObjcFunction &right) const {return getFullName() < right.getFullName();}
        std::string getFullName(void) const;
        std::string getSignature(void) const;
        //bool isObjcFunction(void) const;
        bool isObjcMsgSend(void) const;
        bool isObjcMsgSendSuper(void) const;
        bool isObjcMsgSendSuper2(void) const;
        bool isMethodInvoke(void) const;
        bool isStret(void) const;
        llvm::FunctionCallee getCacheGetImpFunction(llvm::Module &module);
        llvm::FunctionCallee getLookupMethodAndLoadCache3Functions(llvm::Module &module);
        llvm::FunctionCallee getForwardingFunction(llvm::Module &module);
        llvm::FunctionCallee getAbortFunction(llvm::Module &module);
        llvm::Type *getInvokedFunctionType(void);
        llvm::FunctionType *getInvokedFunctionType2(void);

        static char getFunctionSignatureLetter(llvm::Type *type);
        //static bool isObjcFunction(StringRef &name);
        static bool isObjCMsgCalle(const llvm::Function *fn);
        llvm::Function *generateFunction(llvm::Module &module, bool isGSABI2);
    private:
        std::string m_name;
        llvm::FunctionType *m_messageFunctionType;
        // gnustep 2.0
        llvm::Function *generateMsgSendFunction_objc_GS_ABI2(llvm::Module &module);
        llvm::Function *generateMethodInvokeFunction_objc_GS_ABI2(llvm::Module &module);
        // Apple objc4
        llvm::Function *generateMsgSendFunction_objc4(llvm::Module &module);
        llvm::Function *generateMethodInvokeFunction_objc4(llvm::Module &module);
    };*/

    class MsgSendFunction {

    public:
        MsgSendFunction(std::string name, llvm::FunctionType *functionType, MsgSendMethodType type, llvm::Module *module);
        MsgSendFunction(const MsgSendFunction &other) : m_name(other.m_name), m_msgFunctionType(other.m_msgFunctionType), m_msgSendMethod(other.m_msgSendMethod) {};
        ~MsgSendFunction() {};

        bool operator<(const MsgSendFunction &other) const { return this->getWasmFnName() < other.getWasmFnName(); }
        std::string getWasmFnName() const;
        std::string signature() const;
        bool isMethodInvokeCall() const { return this->m_msgSendMethod == MsgSendMethodType::METHOD_INVOKE; }
        bool isStret() const { return this->m_msgSendMethod == MsgSendMethodType::MSG_SEND_ST || this->m_msgSendMethod == MsgSendMethodType::MSG_SEND_FP; }
        MsgSendMethodType dispatchMethodType() const { return this->m_msgSendMethod; }

        llvm::Function *createWrapper(llvm::Module *module);

    private:
        llvm::FunctionType *getInvokedFunctionType() const;
        llvm::Type *getInvokedUnqualType() const;
        llvm::Function *createMsgSendWrapper(llvm::Module *module);
        llvm::Function *createMethodInvokeWrapper(llvm::Module *module);


    private:
        std::string m_name;
        llvm::FunctionType *m_msgFunctionType;
        //bool m_isMethodInvokeCall;
        MsgSendMethodType m_msgSendMethod;
    };

    class MsgSendCallVisitor: public llvm::InstVisitor<MsgSendCallVisitor> {
    public:
        MsgSendCallVisitor(std::map<MsgSendFunction, std::vector<llvm::CallBase*>> &funcs) :funcs(funcs) {};

        void visitCallBase(llvm::CallBase &callInstuction);
    private:
        std::map<MsgSendFunction, std::vector<llvm::CallBase*>> &funcs;
    };
}

char WebAssemblyGenObjCMsgSendFuncs::ID = 0;
INITIALIZE_PASS(WebAssemblyGenObjCMsgSendFuncs, "wasm-generate-objc-message-function",
                                "Generate objective-c message functions for WebAssembly",
                                false, false)

ModulePass *llvm::createWebAssemblyObjCMsgSendFuncsPass() {
    return new WebAssemblyGenObjCMsgSendFuncs();
}

static std::string generateSignature(llvm::FunctionType *fnType);

std::string MsgSendFunction::getWasmFnName(void) const {
    return this->m_name + "_" + generateSignature(this->m_msgFunctionType);
}

static char wasmCharForType(llvm::Type *type)
{
    if (type->isVoidTy()) {
        return 'v';
    } else if (type->isFloatingPointTy()) {

        if (type->isFloatTy()) {
            return 'f';
        } else {
            return 'd';
        }

    } else if (type->isIntegerTy()) {

        if (type->getIntegerBitWidth() == 64) {
            return 'j';
        } else {
            return 'i';
        }

    } else if (type->isPointerTy()) {
        return 'i';
    } else {
        llvm::errs() << "*** Unsupported type!\n";
#if !defined(NDEBUG) || defined(LLVM_ENABLE_DUMP)
        type->dump();
#endif
        return 'x';
    }
}

static std::string generateSignature(llvm::FunctionType *fnType)
{
    std::string signature;
    signature = wasmCharForType(fnType->getReturnType());
    for(auto iter = fnType->param_begin(); iter != fnType->param_end(); ++iter) {
        llvm::Type *type = *iter;
        signature += wasmCharForType(type);
    }

    if (fnType->isVarArg()) {
        signature += "_va";
    }

    return signature;
}



static llvm::Function *getSlowMsgLookupFunction(llvm::Module *module)
{
    // getOrInsertFunction() // FunctionCallee llvm::Module::getOrInsertFunction
    /*llvm::Type *i32Type = llvm::Type::getInt32PtrTy(module.getContext());
    llvm::Type *arguments[] = { i32Type, i32Type };
    return module.getOrInsertFunction("slowMsgLookup", llvm::FunctionType::get(i32Type, arguments, false));*/

    llvm::Function *fn = module->getFunction("slowMsgLookup");
    if (!fn) {
        llvm::Type *i32_ptr_t = llvm::Type::getInt32PtrTy(module->getContext());
        llvm::Type *arguments[] = { i32_ptr_t, i32_ptr_t };
        auto voidFnType = FunctionType::get(Type::getVoidTy(module->getContext()), false); // like declaring void (*fp)(void) ??
        FunctionType *fnType = FunctionType::get(PointerType::getUnqual(voidFnType), arguments, false);
        fn = Function::Create(fnType, Function::ExternalLinkage, "slowMsgLookup", module);
    }
    return fn;
}

//
// Mark: MsgSendFunction impl.
//

MsgSendFunction::MsgSendFunction(std::string name, llvm::FunctionType *functionType, MsgSendMethodType msgSendMethod, llvm::Module *module) : m_name(name), m_msgSendMethod(msgSendMethod) {
    llvm::Type *i32_t = llvm::Type::getInt32PtrTy(module->getContext());

    llvm::SmallVector<llvm::Type*, 10> params;
    int i = 0;
    for(auto it = functionType->param_begin(), E = functionType->param_end(); it != E; ++i, ++it) {
        llvm::Type *type = *it;

        if (type->isPointerTy()) {
            params.push_back(i32_t);
        } else {
            params.push_back(type);
        }
    }

    this->m_msgFunctionType = llvm::FunctionType::get(functionType->getReturnType(), params, functionType->isVarArg());
    //this->m_isMethodInvokeCall = name.rfind(OBJC_METHOD_INVOKE, 0) == 0;
}

llvm::FunctionType* MsgSendFunction::getInvokedFunctionType() const
{
    llvm::FunctionType* functionType;

    if (!this->isMethodInvokeCall()) {
        functionType = m_msgFunctionType;
    } else {
        llvm::SmallVector<llvm::Type*, 10> ArgTypes(m_msgFunctionType->param_begin(), m_msgFunctionType->param_end());
        unsigned idx = !this->isStret() ? 1 : 2;
        ArgTypes[idx] = llvm::Type::getInt32PtrTy(m_msgFunctionType->getContext());
        functionType = llvm::FunctionType::get(m_msgFunctionType->getReturnType(), ArgTypes, false);
    }

    return functionType;
}

llvm::Type* MsgSendFunction::getInvokedUnqualType() const
{
    llvm::FunctionType* functionType;

    if (!this->isMethodInvokeCall()) {
        functionType = m_msgFunctionType;
    } else {
        llvm::SmallVector<llvm::Type*, 10> ArgTypes(m_msgFunctionType->param_begin(), m_msgFunctionType->param_end());
        unsigned idx = !this->isStret() ? 1 : 2;
        ArgTypes[idx] = llvm::Type::getInt32PtrTy(m_msgFunctionType->getContext());
        functionType = llvm::FunctionType::get(m_msgFunctionType->getReturnType(), ArgTypes, m_msgFunctionType->isVarArg());
    }

    return llvm::PointerType::getUnqual(functionType);
}

llvm::Function *MsgSendFunction::createMsgSendWrapper(llvm::Module *module)
{
    using namespace llvm;

    Function *msgSendWrapper = Function::Create(m_msgFunctionType, Function::LinkOnceAnyLinkage, this->getWasmFnName(), module);

    BasicBlock *entryBlock = BasicBlock::Create(module->getContext(), "", msgSendWrapper);
#if 1
    BasicBlock *nilReciverExitBlock = BasicBlock::Create(module->getContext(), "nil-return", msgSendWrapper);
    BasicBlock *lookupBlock = BasicBlock::Create(module->getContext(), "lookup", msgSendWrapper);
    IRBuilder<> builder(entryBlock);
    
    llvm::Type *i32_t = llvm::Type::getInt32Ty(msgSendWrapper->getContext());
    //llvm::Value *arg0 =  new llvm::LoadInst(i32_t, msgSendWrapper->getArg(0), "recivier", lockupBlock);
    //llvm::Type *ptrType = llvm::Type::get
    llvm::Value *receiverAddr = nullptr;
    if (this->isStret()) {
        //msgSendWrapper->getArg(1)->getType()->isPointerTy();
        receiverAddr = msgSendWrapper->getArg(1); // builder.CreateLoad(i32_t, msgSendWrapper->getArg(1), false);
    } else {
        receiverAddr = msgSendWrapper->getArg(0); // builder.CreateLoad(i32_t, msgSendWrapper->getArg(0), false);
    }

    llvm::Value *receiverIsNull = builder.CreateIsNull(receiverAddr);
    builder.CreateCondBr(receiverIsNull, nilReciverExitBlock, lookupBlock);
    
    builder.SetInsertPoint(nilReciverExitBlock);
    llvm::Type *returnType = m_msgFunctionType->getReturnType();
    if (returnType->isVoidTy()) {
        builder.CreateRetVoid();
    } else if (returnType->isFloatTy() || returnType->isDoubleTy()) {
        builder.CreateRet(ConstantFP::get(returnType, 0.0));
    } else if (returnType->isIntegerTy() && returnType->getIntegerBitWidth() == 64) {
        builder.CreateRet(ConstantInt::get(returnType, 0));
    } else {
        builder.CreateRet(ConstantInt::get(i32_t, 0));
    }
    
    Function *slowLockup = getSlowMsgLookupFunction(module);
    llvm::Value *arg0 = nullptr;
    llvm::Value *arg1 = nullptr;
    if (this->isStret()) {
        arg0 = msgSendWrapper->getArg(1);
        arg1 = msgSendWrapper->getArg(2);
    } else {
        arg0 = msgSendWrapper->getArg(0);
        arg1 = msgSendWrapper->getArg(1);
    }
    llvm::Value *lookupArgs[] = {arg0, arg1};
    
    builder.SetInsertPoint(lookupBlock);
    CallInst *lookupCall = builder.CreateCall(slowLockup, lookupArgs);
    
    // TODO: check null-forward etc. might impl objc-forwarding inline here.
    
    auto msgImp = builder.CreateBitOrPointerCast(lookupCall, this->getInvokedUnqualType());
    SmallVector<Value*, 10> actualArgs;
    for(auto &arg: msgSendWrapper->args()) {
        arg.setName("arg");
        actualArgs.push_back(&arg);
    }
    
    llvm::CallInst *msgRet = builder.CreateCall(this->getInvokedFunctionType(), msgImp, actualArgs);
    
    if (returnType->isVoidTy()) {
        builder.CreateRetVoid();
    } else if (returnType->isFloatTy() || returnType->isDoubleTy() || (returnType->isIntegerTy() && returnType->getIntegerBitWidth() == 64)) {
        // Testing if cast works for f32, f64 and i64 returns
        llvm::Value *cast = builder.CreateBitOrPointerCast(msgRet, returnType);
        builder.CreateRet(cast);
    } else {
        builder.CreateRet(msgRet);
    }
    
#endif
#if 0
    // simple test for linux version (which works but not with -wasm-exceptions)
    // which means that there is something that is not working with the code used on macOS when compiled on linux.
    // firstly test to use a builder to chain the code generated. I think NULL check where never realy working in
    // the previous version.
    llvm::Type *i32_t = llvm::Type::getInt32Ty(msgSendWrapper->getContext());
    llvm::Type *returnType = m_msgFunctionType->getReturnType();
    if (returnType->isVoidTy()) {
        ReturnInst::Create(module->getContext(), entryBlock);
    } else if (returnType->isFloatTy() || returnType->isDoubleTy()) {
        // TODO: cast if needed.
        ReturnInst::Create(module->getContext(), ConstantFP::get(returnType, 0.0), entryBlock);
    } else if (returnType->isIntegerTy() && returnType->getIntegerBitWidth() == 64) {
        // TODO: cast if needed.
        ReturnInst::Create(module->getContext(), ConstantExpr::getBitCast(ConstantInt::get(i32_t, 0), returnType), entryBlock);
    } else {
        ReturnInst::Create(module->getContext(), ConstantInt::get(i32_t, 0), entryBlock);
    }

#endif
#if 0
    BasicBlock *nilReciverExitBlock = BasicBlock::Create(module->getContext(), "invoke", msgSendWrapper);
    BasicBlock *checkSmallObjBlock = BasicBlock::Create(module->getContext(), "entry", msgSendWrapper);
    BasicBlock *lockupBlock = BasicBlock::Create(module->getContext(), "lockup", msgSendWrapper);
    BasicBlock *invokeBlock = BasicBlock::Create(module->getContext(), "invoke", msgSendWrapper);
    BasicBlock *returnBlock = BasicBlock::Create(module->getContext(), "return", msgSendWrapper);
    BasicBlock *prepareSmallObjblock = BasicBlock::Create(module->getContext(), "smallObj", msgSendWrapper);
    //BasicBlock *loadClassBlock = BasicBlock::Create(module->getContext(), "smallObj", msgSendWrapper);
    //BasicBlock *dtable_init_block = BasicBlock::Create(module->getContext(), "dtable_init", msgSendWrapper);
    //BasicBlock *dtable_0_block = BasicBlock::Create(module->getContext(), "dtable_0_shift", msgSendWrapper);
    //BasicBlock *dtable_8_block = BasicBlock::Create(module->getContext(), "dtable_8_shift", msgSendWrapper);
    //BasicBlock *dtable_16_Block = BasicBlock::Create(module->getContext(), "dtable_16_shift", msgSendWrapper);
    //BasicBlock *dtable_24_Block = BasicBlock::Create(module->getContext(), "dtable_24_shift", msgSendWrapper);



    llvm::Type *i32_t = llvm::Type::getInt32Ty(msgSendWrapper->getContext());
    bool isVoidReturn = m_msgFunctionType->getReturnType()->isVoidTy();
    const DataLayout &DL = entryBlock->getModule()->getDataLayout();

    // Entry-Point

    //llvm::Value *arg0 =  new llvm::LoadInst(i32_t, msgSendWrapper->getArg(0), "recivier", lockupBlock);
    llvm::Value *arg0 = nullptr;
    if (this->isStret()) {
        arg0 = new llvm::LoadInst(i32_t, msgSendWrapper->getArg(1), "recivier", lockupBlock);
    } else {
        arg0 = new llvm::LoadInst(i32_t, msgSendWrapper->getArg(0), "recivier", lockupBlock);
    }

    if (this->dispatchMethodType() == MsgSendMethodType::MSG_SEND) {
        auto receiverIsNull = new ICmpInst(*entryBlock, CmpInst::ICMP_EQ, arg0, ConstantExpr::getBitCast(ConstantInt::get(i32_t, 0), i32_t), "null-receiver");
        BranchInst::Create(nilReciverExitBlock, lockupBlock/*checkSmallObjBlock*/, receiverIsNull, entryBlock);
    } else {
        BranchInst::Create(lockupBlock, entryBlock);
    }

    //auto receiverIsSmallObj = new ICmpInst(*checkSmallObjBlock, CmpInst::ICMP_EQ, arg0, ConstantExpr::getBitCast(ConstantInt::get(i32_t, 0), i32_t), "receiver-is-small-object");
    //BranchInst::Create(prepareSmallObjblock, invokeBlock, receiverIsSmallObj, checkSmallObjBlock); // TODO: should goto dtable_lockup

    // Load Class into register.
    //llvm::Value *cls = llvm::GetElementPtrInst::CreateInBounds(i32_t, arg0, {ConstantInt::get(i32_t, 0), ConstantInt::get(i32_t, 0)}, "self.cls", loadClassBlock);
    //BranchInst::Create(dtable_init_block, loadClassBlock); // jump to dtable lookup.


    // dtable lookup.



    // Prepare Small Object

    // Determine what arguments to pass.
    /*SmallVector<Value *, 4> Args;
    Function::arg_iterator AI = msgSendWrapper->arg_begin();
    Function::arg_iterator AE = msgSendWrapper->arg_end();
    FunctionType::param_iterator PI = F->getFunctionType()->param_begin();
    FunctionType::param_iterator PE = F->getFunctionType()->param_end();*/
  /*
  Type *ExpectedRtnType = F->getFunctionType()->getReturnType();
  Type *RtnType = Ty->getReturnType();

  if ((F->getFunctionType()->getNumParams() != Ty->getNumParams()) ||
      (F->getFunctionType()->isVarArg() != Ty->isVarArg()) ||
      (ExpectedRtnType != RtnType))
    WrapperNeeded = true;

  for (; AI != AE && PI != PE; ++AI, ++PI) {
    Type *ArgType = AI->getType();
    Type *ParamType = *PI;

    if (ArgType == ParamType) {
      Args.push_back(&*AI);
    } else {
      if (CastInst::isBitOrNoopPointerCastable(ArgType, ParamType, DL)) {
        Instruction *PtrCast =
            CastInst::CreateBitOrPointerCast(AI, ParamType, "cast");
        BB->getInstList().push_back(PtrCast);
        Args.push_back(PtrCast);
      } else if (ArgType->isStructTy() || ParamType->isStructTy()) {
        LLVM_DEBUG(dbgs() << "createMsgSendWrapper: struct param type in bitcast: "
                          << F->getName() << "\n");
        WrapperNeeded = false;
      } else {
        LLVM_DEBUG(dbgs() << "createMsgSendWrapper: arg type mismatch calling: "
                          << F->getName() << "\n");
        LLVM_DEBUG(dbgs() << "Arg[" << Args.size() << "] Expected: "
                          << *ParamType << " Got: " << *ArgType << "\n");
        TypeMismatch = true;
        break;
      }
    }
  }

  if (WrapperNeeded && !TypeMismatch) {
    for (; PI != PE; ++PI)
      Args.push_back(UndefValue::get(*PI));
    if (F->isVarArg())
      for (; AI != AE; ++AI)
        Args.push_back(&*AI);*/

    // null reciver Exit
    llvm::Type *returnType = m_msgFunctionType->getReturnType();
    if (returnType->isVoidTy()) {
        ReturnInst::Create(module->getContext(), nilReciverExitBlock);
    } else if (returnType->isFloatTy() || returnType->isDoubleTy()) {
        // TODO: cast if needed.
        ReturnInst::Create(module->getContext(), ConstantFP::get(returnType, 0.0), nilReciverExitBlock);
    } else if (returnType->isIntegerTy() && returnType->getIntegerBitWidth() == 64) {
        // TODO: cast if needed.
        ReturnInst::Create(module->getContext(), ConstantExpr::getBitCast(ConstantInt::get(i32_t, 0), returnType), nilReciverExitBlock);
    } else {
        ReturnInst::Create(module->getContext(), ConstantInt::get(i32_t, 0), nilReciverExitBlock);
    }

    // Slow-Msg-Lockup Block

    Function *slowLockup = getSlowMsgLookupFunction(module);

    //arg0 = new llvm::LoadInst(i32_t, msgSendWrapper->getArg(0), "recivier", lockupBlock);
    //llvm::Value *arg1 = new llvm::LoadInst(i32_t, msgSendWrapper->getArg(1), "selector", lockupBlock);
    llvm::Value *arg1 = nullptr;
    if (this->isStret()) {
        arg0 = msgSendWrapper->getArg(1);
        arg1 = msgSendWrapper->getArg(2);
    } else {
        arg0 = msgSendWrapper->getArg(0);
        arg1 = msgSendWrapper->getArg(1);
    }
    llvm::Value *lockupArgs[] = {arg0, arg1};

    CallInst *Call = CallInst::Create(slowLockup, lockupArgs, "", lockupBlock);
    BranchInst::Create(invokeBlock, lockupBlock);

    // Invoke Block
    auto actualFn = new BitCastInst(Call, this->getInvokedUnqualType(), "imp_cast", invokeBlock);
    SmallVector<Value*, 10> actualArgs;
    for(auto &arg: msgSendWrapper->args()) {
        arg.setName("arg");
        actualArgs.push_back(&arg);
    }
    llvm::CallInst *impCallRet = CallInst::Create(this->getInvokedFunctionType(), actualFn, actualArgs, isVoidReturn ? "" : "call_ret", invokeBlock);
    BranchInst::Create(returnBlock, invokeBlock);

    // Return Block (return type is already set in nil-exit block above)
    if (returnType->isVoidTy()) {
        ReturnInst::Create(module->getContext(), returnBlock);
    } else if (returnType->isFloatTy() || returnType->isDoubleTy() || (returnType->isIntegerTy() && returnType->getIntegerBitWidth() == 64)) {
        // Testing if cast works for f32, f64 and i64 returns
        Instruction *cast = CastInst::CreateBitOrPointerCast(impCallRet, returnType, "cast");
        returnBlock->getInstList().push_back(cast);
        ReturnInst::Create(module->getContext(), cast, returnBlock);/*
    } else if (returnType->isIntegerTy() && returnType->getIntegerBitWidth() == 64) {
        // TODO: cast if needed.
        ReturnInst::Create(module->getContext(), impCallRet, returnBlock);*/
    } else {
        ReturnInst::Create(module->getContext(), impCallRet, returnBlock);
    }
    //}
    /*
    if (TypeMismatch) {
        // Create a new wrapper that simply contains `unreachable`.
        msgSendWrapper->eraseFromParent();
        msgSendWrapper = Function::Create(Ty, Function::PrivateLinkage, F->getName() + "_bitcast_invalid", module);
        BasicBlock *BB = BasicBlock::Create(module->getContext(), "body", msgSendWrapper);
        new UnreachableInst(module->getContext(), BB);
        msgSendWrapper->setName(F->getName() + "_bitcast_invalid");
    } else if (!WrapperNeeded) {
        LLVM_DEBUG(dbgs() << "createMsgSendWrapper: no wrapper needed: " << F->getName() << "\n");
        msgSendWrapper->eraseFromParent();
        return nullptr;
    }*/
#endif
    LLVM_DEBUG(dbgs() << "createMsgSendWrapper: " << msgSendWrapper->getName() << "\n");
    return msgSendWrapper;
}

llvm::Function *MsgSendFunction::createMethodInvokeWrapper(llvm::Module *module)
{
    Function *invokeWrapper = Function::Create(m_msgFunctionType, Function::LinkOnceAnyLinkage, this->getWasmFnName(), module);
    BasicBlock *invokeBlock = BasicBlock::Create(module->getContext(), "invoke", invokeWrapper);
    BasicBlock *returnBlock = BasicBlock::Create(module->getContext(), "return", invokeWrapper);
    const DataLayout &DL = invokeBlock->getModule()->getDataLayout();

    // Determine what arguments to pass.
    /*SmallVector<Value *, 4> Args;
    Function::arg_iterator AI = msgSendWrapper->arg_begin();
    Function::arg_iterator AE = msgSendWrapper->arg_end();
    FunctionType::param_iterator PI = F->getFunctionType()->param_begin();
    FunctionType::param_iterator PE = F->getFunctionType()->param_end();*/
    bool isVoidReturn = m_msgFunctionType->getReturnType()->isVoidTy();
  /*
  Type *ExpectedRtnType = F->getFunctionType()->getReturnType();
  Type *RtnType = Ty->getReturnType();

  if ((F->getFunctionType()->getNumParams() != Ty->getNumParams()) ||
      (F->getFunctionType()->isVarArg() != Ty->isVarArg()) ||
      (ExpectedRtnType != RtnType))
    WrapperNeeded = true;

  for (; AI != AE && PI != PE; ++AI, ++PI) {
    Type *ArgType = AI->getType();
    Type *ParamType = *PI;

    if (ArgType == ParamType) {
      Args.push_back(&*AI);
    } else {
      if (CastInst::isBitOrNoopPointerCastable(ArgType, ParamType, DL)) {
        Instruction *PtrCast =
            CastInst::CreateBitOrPointerCast(AI, ParamType, "cast");
        BB->getInstList().push_back(PtrCast);
        Args.push_back(PtrCast);
      } else if (ArgType->isStructTy() || ParamType->isStructTy()) {
        LLVM_DEBUG(dbgs() << "createMsgSendWrapper: struct param type in bitcast: "
                          << F->getName() << "\n");
        WrapperNeeded = false;
      } else {
        LLVM_DEBUG(dbgs() << "createMsgSendWrapper: arg type mismatch calling: "
                          << F->getName() << "\n");
        LLVM_DEBUG(dbgs() << "Arg[" << Args.size() << "] Expected: "
                          << *ParamType << " Got: " << *ArgType << "\n");
        TypeMismatch = true;
        break;
      }
    }
  }

  if (WrapperNeeded && !TypeMismatch) {
    for (; PI != PE; ++PI)
      Args.push_back(UndefValue::get(*PI));
    if (F->isVarArg())
      for (; AI != AE; ++AI)
        Args.push_back(&*AI);*/

    llvm::Type *i32_t = llvm::Type::getInt32Ty(invokeWrapper->getContext());

    llvm::Value *imp = invokeWrapper->getArg(1);

    // Invoke Block
    auto actualFn = new BitCastInst(imp, this->getInvokedUnqualType(), "imp_cast", invokeBlock);

    SmallVector<Value*, 10> actualArgs;
    for(auto &arg: invokeWrapper->args()) {
        arg.setName("arg");
        actualArgs.push_back(&arg);
    }
    actualArgs[1] = ConstantPointerNull::get(PointerType::get(i32_t, 0)); // replace imp with nil selector for call.

    llvm::CallInst *impCallRet = CallInst::Create(this->getInvokedFunctionType(), actualFn, actualArgs, isVoidReturn ? "" : "call_ret", invokeBlock);
    BranchInst::Create(returnBlock, invokeBlock);

    // Determine what value to return.
    if (isVoidReturn) {
        ReturnInst::Create(module->getContext(), returnBlock);
    } else {
        ReturnInst::Create(module->getContext(), impCallRet, returnBlock);
    }
    //}
    /*
    if (TypeMismatch) {
        // Create a new wrapper that simply contains `unreachable`.
        msgSendWrapper->eraseFromParent();
        msgSendWrapper = Function::Create(Ty, Function::PrivateLinkage, F->getName() + "_bitcast_invalid", module);
        BasicBlock *BB = BasicBlock::Create(module->getContext(), "body", msgSendWrapper);
        new UnreachableInst(module->getContext(), BB);
        msgSendWrapper->setName(F->getName() + "_bitcast_invalid");
    } else if (!WrapperNeeded) {
        LLVM_DEBUG(dbgs() << "createMsgSendWrapper: no wrapper needed: " << F->getName() << "\n");
        msgSendWrapper->eraseFromParent();
        return nullptr;
    }*/
    LLVM_DEBUG(dbgs() << "createMsgSendWrapper: " << invokeWrapper->getName() << "\n");
    return invokeWrapper;
}

Function *MsgSendFunction::createWrapper(Module *module) {

    if (this->isMethodInvokeCall()) {
        return this->createMethodInvokeWrapper(module);
    } else {
        return this->createMsgSendWrapper(module);
    }
}

static MsgSendMethodType isObjCMsgCalle(const llvm::Function *fn)
{
    llvm::StringRef name = fn->getName();
    if (name.startswith(llvm::StringRef(OBJC_MSG_SEND_ST))) {
        return MsgSendMethodType::MSG_SEND_ST;
    } else if (name.startswith(llvm::StringRef(OBJC_MSG_SEND_FP))) {
        return MsgSendMethodType::MSG_SEND_FP;
    } else if (name.startswith(llvm::StringRef(OBJC_MSG_SEND))) {
        return MsgSendMethodType::MSG_SEND;
    } else if (name.startswith(llvm::StringRef(OBJC_METHOD_INVOKE))) {
        return MsgSendMethodType::METHOD_INVOKE;
    }

    return MsgSendMethodType::NONE;
}

static bool isObjCMethodInvokeCalle(const llvm::Function *fn)
{
    llvm::StringRef name = fn->getName();
    return name.startswith(llvm::StringRef(OBJC_METHOD_INVOKE));
}

// Recursively descend the def-use lists from V to find non-bitcast users of
// bitcasts of V.
static void findUses(Value *V, Function &F, SmallVectorImpl<std::pair<CallBase *, Function *>> &Uses) {

    for (User *U : V->users()) {
        if (auto *BC = dyn_cast<BitCastOperator>(U))
            findUses(BC, F, Uses);
        else if (auto *A = dyn_cast<GlobalAlias>(U))
            findUses(A, F, Uses);
        else if (auto *CB = dyn_cast<CallBase>(U)) {
            Value *Callee = CB->getCalledOperand();
            if (Callee != V)
                // Skip calls where the function isn't the callee
                continue;
            if (CB->getFunctionType() == F.getValueType())
                // Skip uses that are immediately called
                continue;
            Uses.push_back(std::make_pair(CB, &F));
        }
    }
}

void MsgSendCallVisitor::visitCallBase(CallBase &cb) {
    const Function *fn = cb.getCalledFunction();
    if (!fn) {
        auto *op = cb.getCalledOperand();
        op = op->stripPointerCasts();
        fn = dyn_cast<const Function>(op);
    }
    if (!fn) {
        return;
    }

    MsgSendMethodType kind = isObjCMsgCalle(fn);

    if (kind == MsgSendMethodType::NONE) {
        return;
    }

    MsgSendFunction function(fn->getName().str(), cb.getFunctionType(), kind, cb.getParent()->getModule());

    funcs[function].push_back(&cb);
}

static void removeFunctionIfExist(llvm::Module &module, std::string name) {
    llvm::Function *fn = module.getFunction(name);
    if (fn) {
        fn->eraseFromParent();
    }
}

static void removeFunctions(llvm::Module &module)
{
    removeFunctionIfExist(module, OBJC_MSG_SEND);
    removeFunctionIfExist(module, OBJC_MSG_SEND + "_stret");
    removeFunctionIfExist(module, OBJC_MSG_SEND + "_fpret");
    removeFunctionIfExist(module, OBJC_METHOD_INVOKE);
    removeFunctionIfExist(module, OBJC_METHOD_INVOKE + "_stret");
}

bool WebAssemblyGenObjCMsgSendFuncs::runOnModule(Module &module) {
    LLVM_DEBUG(dbgs() << "********** Fix objc dispatch methods Bitcasts **********\n");

    /*
    SmallVector<std::pair<CallBase *, Function *>, 0> Uses;

    // Collect all the places that need wrappers.
    for (Function &F : M) {
        // Skip to fix when the function is swiftcc because swiftcc allows
        // bitcast type difference for swiftself and swifterror.
        if (F.getCallingConv() == CallingConv::Swift)
            continue;
        if (!isObjCMsgCalle(&F))
            continue;

        findUses(&F, F, Uses);
    }

    DenseMap<std::pair<Function *, FunctionType *>, Function *> Wrappers;

    for (auto &UseFunc : Uses) {
        CallBase *CB = UseFunc.first;
        Function *F = UseFunc.second;
        FunctionType *Ty = CB->getFunctionType();

        auto Pair = Wrappers.insert(std::make_pair(std::make_pair(F, Ty), nullptr));
        if (Pair.second)
        Pair.first->second = createMsgSendWrapper(F, Ty);

        Function *Wrapper = Pair.first->second;
        if (!Wrapper)
        continue;

        CB->setCalledOperand(Wrapper);
    }

  return true;*/

    std::map<MsgSendFunction, std::vector<llvm::CallBase*>> funcs;
    MsgSendCallVisitor visitor(funcs);
    visitor.visit(module);

    //printf("did run WebAssemblyGenObjCMsgSendFuncs::runOnModule\n");

    for(auto it = funcs.begin(), it_end = funcs.end(); it != it_end; ++it) {
        MsgSendFunction func = it->first;
        llvm::Function *replacemntFn = func.createWrapper(&module);
        std::vector<llvm::CallBase*> calls = it->second;
        for(auto it2 = calls.begin(), end = calls.end(); it2 != end; ++it2) {
            llvm::CallBase *cb = *it2;
            //auto cf = ConstantExpr::getBitCast(replacemntFn, PointerType::getUnqual(cb->getFunctionType()));
            //cb->setCalledFunction(replacemntFn);
            cb->setCalledOperand(replacemntFn);
        }
    }

    removeFunctions(module);

    return true;
}



#if 0

namespace {
    const std::string OBJC_MSG_SEND("objc_msgSend");
    const std::string OBJC_MSG_SEND_SUPER("objc_msgSendSuper");
    const std::string OBJC_MSG_SEND_SUPER2("objc_msgSendSuper2");
    const std::string OBJC_METHOD_INVOKE("method_invoke");

class WebAssemblyGenObjCMsgSendFuncs : public llvm::ModulePass {
    public:
        static char ID;
        WebAssemblyGenObjCMsgSendFuncs() : llvm::ModulePass(ID) {
            initializeWebAssemblyGenObjCMsgSendFuncsPass(*llvm::PassRegistry::getPassRegistry());
        }

        virtual bool runOnModule(llvm::Module &module);
        void removeFunctions(llvm::Module &module);
        void removeFunctionIfExist(llvm::Module &module, std::string name);
    };

    class ObjcFunction {
    public:
        ObjcFunction(std::string name, llvm::FunctionType *messageFunctionType, llvm::Module *module);
        ObjcFunction(const ObjcFunction& other) : m_name(other.m_name), m_messageFunctionType(other.m_messageFunctionType) {};
        ~ObjcFunction() {};

        bool operator<(const ObjcFunction &right) const {return getFullName() < right.getFullName();}
        std::string getFullName(void) const;
        std::string getSignature(void) const;
        //bool isObjcFunction(void) const;
        bool isObjcMsgSend(void) const;
        bool isObjcMsgSendSuper(void) const;
        bool isObjcMsgSendSuper2(void) const;
        bool isMethodInvoke(void) const;
        bool isStret(void) const;
        llvm::FunctionCallee getCacheGetImpFunction(llvm::Module &module);
        llvm::FunctionCallee getLookupMethodAndLoadCache3Functions(llvm::Module &module);
        llvm::FunctionCallee getForwardingFunction(llvm::Module &module);
        llvm::FunctionCallee getAbortFunction(llvm::Module &module);
        llvm::Type *getInvokedFunctionType(void);
        llvm::FunctionType *getInvokedFunctionType2(void);

        static char getFunctionSignatureLetter(llvm::Type *type);
        //static bool isObjcFunction(StringRef &name);
        static bool isObjCMsgCalle(const llvm::Function *fn);
        llvm::Function *generateFunction(llvm::Module &module, bool isGSABI2);
    private:
        std::string m_name;
        llvm::FunctionType *m_messageFunctionType;
        // gnustep 2.0
        llvm::Function *generateMsgSendFunction_objc_GS_ABI2(llvm::Module &module);
        llvm::Function *generateMethodInvokeFunction_objc_GS_ABI2(llvm::Module &module);
        // Apple objc4
        llvm::Function *generateMsgSendFunction_objc4(llvm::Module &module);
        llvm::Function *generateMethodInvokeFunction_objc4(llvm::Module &module);
    };

class ObjcCallVisitor: public llvm::InstVisitor<ObjcCallVisitor> {
    public:
        ObjcCallVisitor(std::map<ObjcFunction, std::vector<llvm::CallBase*>> &funcs) :funcs(funcs) {};

        void visitCallInst(llvm::CallInst &callInstuction);
        void visitInvokeInst(llvm::InvokeInst &invokeInstuction);

        //void handleCall(Instruction *CI);
    private:
        std::map<ObjcFunction, std::vector<llvm::CallBase*>> &funcs;
    };
}

char WebAssemblyGenObjCMsgSendFuncs::ID = 0;
INITIALIZE_PASS(WebAssemblyGenObjCMsgSendFuncs, "wasm-generate-objc-message-function",
                                "Generate objective-c message functions for WebAssembly",
                                false, false)

bool WebAssemblyGenObjCMsgSendFuncs::runOnModule(llvm::Module &module) {
    std::map<ObjcFunction, std::vector<llvm::CallBase*>> funcs;
    ObjcCallVisitor visitor(funcs);
    visitor.visit(module);

    //printf("did run WebAssemblyGenObjCMsgSendFuncs::runOnModule\n");

    for(auto it = funcs.begin(), it_end = funcs.end(); it != it_end; ++it) {
        ObjcFunction func = it->first;
        llvm::Function *replacemntFn = func.generateFunction(module, true);
        std::vector<llvm::CallBase*> calls = it->second;
        for(auto it2 = calls.begin(), end = calls.end(); it2 != end; ++it2) {
            llvm::CallBase *cb = *it2;
            //auto cf = ConstantExpr::getBitCast(replacemntFn, PointerType::getUnqual(cb->getFunctionType()));
            cb->setCalledFunction(replacemntFn);
        }
    }

    removeFunctions(module);

    return true;
}

void WebAssemblyGenObjCMsgSendFuncs::removeFunctions(llvm::Module &module)
{
    this->removeFunctionIfExist(module, OBJC_MSG_SEND);
    this->removeFunctionIfExist(module, OBJC_MSG_SEND_SUPER);
    this->removeFunctionIfExist(module, OBJC_MSG_SEND_SUPER2);
    this->removeFunctionIfExist(module, OBJC_METHOD_INVOKE);
    this->removeFunctionIfExist(module, OBJC_MSG_SEND + "_stret");
    this->removeFunctionIfExist(module, OBJC_MSG_SEND_SUPER + "_stret");
    this->removeFunctionIfExist(module, OBJC_MSG_SEND_SUPER2 + "_stret");
    this->removeFunctionIfExist(module, OBJC_METHOD_INVOKE + "_stret");
}

void WebAssemblyGenObjCMsgSendFuncs::removeFunctionIfExist(llvm::Module &module, std::string name) {
    auto fn = module.getFunction(name);
    if (fn) {
        fn->eraseFromParent();
    }
}

ObjcFunction::ObjcFunction(std::string name, llvm::FunctionType *functionType, llvm::Module *module) : m_name(name) {
    llvm::Type *i8Type = llvm::Type::getInt8PtrTy(module->getContext());

    llvm::SmallVector<llvm::Type*, 10> arguments;
    int i = 0;
    for(auto it = functionType->param_begin(), E = functionType->param_end(); it != E; ++i, ++it) {
        llvm::Type *type = *it;

        // keep objcSuper
        bool keep = false;
        if (isObjcMsgSendSuper() || isObjcMsgSendSuper2()) {
            if (!isStret() && i == 0)
                keep = true;
            if (isStret() && i == 1)
                keep = true;
        }

        if (!keep && type->isPointerTy()) {
            arguments.push_back(i8Type);
        } else {
            arguments.push_back(type);
        }
    }

    this->m_messageFunctionType = llvm::FunctionType::get(functionType->getReturnType(), arguments, false);
}

std::string ObjcFunction::getFullName(void) const {
    return this->m_name + "_" + this->getSignature();
}

std::string ObjcFunction::getSignature(void) const
{
    std::string signature;
    signature = getFunctionSignatureLetter(m_messageFunctionType->getReturnType());
    for(auto iter = m_messageFunctionType->param_begin(); iter != m_messageFunctionType->param_end(); ++iter) {
        llvm::Type *type = *iter;
        signature += ObjcFunction::getFunctionSignatureLetter(type);
    }

    return signature;
}

char ObjcFunction::getFunctionSignatureLetter(llvm::Type *type)
{
    if (type->isVoidTy()) {
        return 'v';
    } else if (type->isFloatingPointTy()) {

        if (type->isFloatTy()) {
            return 'f';
        } else {
            return 'd';
        }

    } else if (type->isIntegerTy()) {

        if (type->getIntegerBitWidth() == 64) {
            return 'j';
        } else {
            return 'i';
        }

    } else if (type->isPointerTy()) {
        return 'i';
    } else {
        llvm::errs() << "*** Unsupported type!\n";
#if !defined(NDEBUG) || defined(LLVM_ENABLE_DUMP)
        type->dump();
#endif
        return 'x';
    }
}

bool ObjcFunction::isObjCMsgCalle(const llvm::Function *fn)
{
    llvm::StringRef name = fn->getName();
    return name.startswith(llvm::StringRef(OBJC_MSG_SEND)) || name.startswith(llvm::StringRef(OBJC_MSG_SEND_SUPER))  || name.startswith(llvm::StringRef(OBJC_MSG_SEND_SUPER2)) || name.startswith(llvm::StringRef(OBJC_METHOD_INVOKE));
}

/*
bool ObjcFunction::isObjcFunction(StringRef name)
{
    return name.startswith(StringRef(OBJC_MSG_SEND))
        || name.startswith(StringRef(OBJC_MSG_SEND_SUPER))
        || name.startswith(StringRef(OBJC_MSG_SEND_SUPER2))
        || name.startswith(StringRef(OBJC_METHOD_INVOKE));*/

    /*return name.compare(0, OBJC_MSG_SEND.length(), OBJC_MSG_SEND) == 0
        || name.compare(0, OBJC_MSG_SEND_SUPER.length(), OBJC_MSG_SEND_SUPER) == 0
        || name.compare(0, OBJC_MSG_SEND_SUPER2.length(), OBJC_MSG_SEND_SUPER2) == 0
        || name.compare(0, OBJC_METHOD_INVOKE.length(), OBJC_METHOD_INVOKE) == 0;*/
//}


bool ObjcFunction::isObjcMsgSend(void) const
{
    return this->m_name == OBJC_MSG_SEND || this->m_name == (OBJC_MSG_SEND + "_stret");
}

bool ObjcFunction::isObjcMsgSendSuper(void) const
{
    return this->m_name == OBJC_MSG_SEND_SUPER || this->m_name == (OBJC_MSG_SEND_SUPER + "_stret");
}

bool ObjcFunction::isObjcMsgSendSuper2(void) const
{
    return this->m_name == OBJC_MSG_SEND_SUPER2 || this->m_name == (OBJC_MSG_SEND_SUPER2 + "_stret");
}

bool ObjcFunction::isMethodInvoke(void) const
{
    return this->m_name == OBJC_METHOD_INVOKE || this->m_name == (OBJC_METHOD_INVOKE + "_stret");
}

bool ObjcFunction::isStret(void) const
{
    return this->m_name.find("_stret") != std::string::npos;
}

llvm::FunctionCallee ObjcFunction::getCacheGetImpFunction(llvm::Module &module)
{
    // getOrInsertFunction() // FunctionCallee llvm::Module::getOrInsertFunction
    llvm::Type *i8Type = llvm::Type::getInt8PtrTy(module.getContext());
    llvm::Type *arguments[] = { i8Type, i8Type };
    llvm::FunctionType *VoidFuncType = llvm::FunctionType::get(llvm::Type::getVoidTy(module.getContext()), false);
    return module.getOrInsertFunction("cache_getImp", llvm::FunctionType::get(llvm::PointerType::getUnqual(VoidFuncType), arguments, false));
    /*
    auto fn = module.getFunction("cache_getImp");
    if (!fn) {
        Type *i8Type = Type::getInt8PtrTy(module.getContext());
        Type *arguments[] = { i8Type, i8Type };
        auto VoidFuncType = FunctionType::get(Type::getVoidTy(module.getContext()), false);
        auto CacheGetImpFuncType = FunctionType::get(PointerType::getUnqual(VoidFuncType), arguments, false);
        fn = Function::Create(CacheGetImpFuncType, GlobalValue::ExternalLinkage, "cache_getImp", &module);
    }
    return fn;
    */
}

llvm::FunctionCallee ObjcFunction::getLookupMethodAndLoadCache3Functions(llvm::Module &module)
{
    llvm::Type *i8Type = llvm::Type::getInt8PtrTy(module.getContext());
    llvm::Type *arguments[] = { i8Type, i8Type, i8Type };
    llvm::FunctionType *VoidFuncType = llvm::FunctionType::get(llvm::Type::getVoidTy(module.getContext()), false);
    return module.getOrInsertFunction("_class_lookupMethodAndLoadCache3", llvm::FunctionType::get(llvm::PointerType::getUnqual(VoidFuncType), arguments, false));
    /*
    auto fn = module.getFunction("_class_lookupMethodAndLoadCache3");
    if (!fn) {
        Type *i8Type = Type::getInt8PtrTy(module.getContext());
        Type *arguments[] = { i8Type, i8Type, i8Type };
        auto VoidFuncType = FunctionType::get(Type::getVoidTy(module.getContext()), false);
        auto LookupFuncType = FunctionType::get(PointerType::getUnqual(VoidFuncType), arguments, false);
        fn = Function::Create(LookupFuncType, GlobalValue::ExternalLinkage, "_class_lookupMethodAndLoadCache3", &module);
    }
    return fn;*/
}

llvm::FunctionCallee ObjcFunction::getForwardingFunction(llvm::Module &module)
{
    llvm::Type *i8Type = llvm::Type::getInt8PtrTy(module.getContext());
    llvm::Type *arguments[] = { i8Type, i8Type };
    return module.getOrInsertFunction("__forwarding__", llvm::FunctionType::get(i8Type, arguments, false));
    /*
    auto fn = module.getFunction("__forwarding__");
    if (!fn) {
        Type *i8Type = Type::getInt8PtrTy(module.getContext());
        Type *arguments[] = { i8Type, i8Type };
        auto CacheGetImpFuncType = FunctionType::get(i8Type, arguments, false);
        fn = Function::Create(CacheGetImpFuncType, GlobalValue::ExternalLinkage, "__forwarding__", &module);
    }
    return fn;
    */
}

llvm::FunctionCallee ObjcFunction::getAbortFunction(llvm::Module &module)
{
    llvm::FunctionType *VoidFuncType = llvm::FunctionType::get(llvm::Type::getVoidTy(module.getContext()), false);
    return module.getOrInsertFunction("abort", llvm::FunctionType::get(VoidFuncType, false));
    /*
    auto fn = module.getFunction("abort");
    if (!fn) {
        auto VoidFuncType = FunctionType::get(Type::getVoidTy(module.getContext()), false);
        fn = Function::Create(VoidFuncType, GlobalValue::ExternalLinkage, "abort", &module);
        fn->addFnAttr(Attribute::NoReturn);
    }
    return fn;
    */
}

// objc_msgSuper/Super2:        objcSuper, SEL, ...           -> self, SEL, ...
// objc_msgSuper/Super2_stret:  staddr, objcSuper, SEL, ...   -> staddr, self, SEL, ...
// method_invoke:               target, method, ...           -> target, SEL, ...
// method_invoke_stret:         staddr, target, method, ...   -> staddr, target, SEL, ...
llvm::Type *ObjcFunction::getInvokedFunctionType(void)
{
    llvm::FunctionType* functionType;

    if (isObjcMsgSend()) {
        functionType = m_messageFunctionType;
    } else {
        llvm::SmallVector<llvm::Type*, 10> ArgTypes(m_messageFunctionType->param_begin(), m_messageFunctionType->param_end());
        unsigned idx;
        if (isObjcMsgSendSuper() || isObjcMsgSendSuper2()) {
            idx = !isStret() ? 0 : 1;
        } else {
            idx = !isStret() ? 1 : 2;
        }
        ArgTypes[idx] = llvm::Type::getInt8PtrTy(m_messageFunctionType->getContext());
        functionType = llvm::FunctionType::get(m_messageFunctionType->getReturnType(), ArgTypes, false);
    }

    return llvm::PointerType::getUnqual(functionType);
}

llvm::FunctionType *ObjcFunction::getInvokedFunctionType2(void)
{
    if (isObjcMsgSend()) {
        return m_messageFunctionType;
    } else {
        llvm::SmallVector<llvm::Type*, 10> ArgTypes(m_messageFunctionType->param_begin(), m_messageFunctionType->param_end());
        unsigned idx;
        if (isObjcMsgSendSuper() || isObjcMsgSendSuper2()) {
            idx = !isStret() ? 0 : 1;
        } else {
            idx = !isStret() ? 1 : 2;
        }
        ArgTypes[idx] = llvm::Type::getInt8PtrTy(m_messageFunctionType->getContext());
        return llvm::FunctionType::get(m_messageFunctionType->getReturnType(), ArgTypes, false);
    }
}



llvm::Function *ObjcFunction::generateFunction(llvm::Module &module, bool isGSABI2)
{
    //LLVM_DEBUG(llvm::dbgs() << "generateFunction: " << getFullName() << "\n");
    //printf("generateFunction: %s\n", this->getFullName().c_str());
    if (isGSABI2) {

        if (isMethodInvoke()) {
            return this->generateMethodInvokeFunction_objc_GS_ABI2(module);
        } else {
            return this->generateMsgSendFunction_objc_GS_ABI2(module);
        }

    } else {

        if (isMethodInvoke()) {
            return generateMethodInvokeFunction_objc4(module);
        } else {
            return generateMsgSendFunction_objc4(module);
        }
    }
}

// mark: gnustep ABI 2.0

// Generates IR instructions
llvm::Function *ObjcFunction::generateMsgSendFunction_objc_GS_ABI2(llvm::Module &module)
{
    //printf("generateMsgSendFunction_objc_GS_ABI2: %s\n", this->getFullName().c_str());
    using namespace llvm;
    Function* Func = Function::Create(m_messageFunctionType, GlobalValue::ExternalLinkage, this->getFullName(), &module);
    /*
    // common Type
    Type *ir_i32_t = Type::getInt32Ty(Func->getContext());
    Type *return_t = m_messageFunctionType->getReturnType();
    Value *Index1[] = {ConstantInt::get(ir_i32_t, 1)};
    Value *Index00[] = {ConstantInt::get(ir_i32_t, 0), ConstantInt::get(ir_i32_t, 0)};
    Value *Index01[] = {ConstantInt::get(ir_i32_t, 0), ConstantInt::get(ir_i32_t, 1)};
    bool isVoidReturn = return_t->isVoidTy();

    // Prepare blocks
    BasicBlock *entryBlock = BasicBlock::Create(Func->getContext(), "", Func);
    BasicBlock *cacheBlock = BasicBlock::Create(Func->getContext(), "Cache", Func);
    BasicBlock *lookupBlock = BasicBlock::Create(Func->getContext(), "Lookup", Func);
    BasicBlock *checkBlock = BasicBlock::Create(Func->getContext(), "Check", Func);
    BasicBlock *callBlock = BasicBlock::Create(Func->getContext(), "Call", Func);
    BasicBlock *forwardBlock = BasicBlock::Create(Func->getContext(), "Foward", Func);
    BasicBlock *retargetBlock = BasicBlock::Create(Func->getContext(), "Retarget", Func);
    BasicBlock *forwardReturnBlock = isVoidReturn ? nullptr : BasicBlock::Create(Func->getContext(), "ForwardReturn", Func);
    BasicBlock *returnBlock = BasicBlock::Create(Func->getContext(), "Return", Func);


    // Setup arguments
    SmallVector<Value*, 10> arguments;
    for(auto &Arg: Func->args()) {
        Arg.setName("arg");
        arguments.push_back(&Arg);
    }

    auto ArgIter = Func->arg_begin();
    Argument *FisrtArg = &*ArgIter++;
    Argument *SecondArg = &*ArgIter++;
    Argument *ThirdArg = isStret() ? &*ArgIter : nullptr;

    Value *self_v;
    Value *super_v;
    Value *st_addr_v;
    Value *sel_v;

    if (isObjcMsgSend()) {
        if (!isStret()) {
            FisrtArg->setName("self");
            SecondArg->setName("sel");
            self_v = FisrtArg;
            sel_v = SecondArg;
        } else {
            FisrtArg->setName("staddr");
            SecondArg->setName("self");
            ThirdArg->setName("sel");
            st_addr_v = FisrtArg;
            self_v = SecondArg;
            sel_v = ThirdArg;
        }
    } else { // Super or Super2
        if (!isStret()) {
            FisrtArg->setName("objcSuper");
            SecondArg->setName("sel");
            super_v = FisrtArg;
            sel_v = SecondArg;
        } else {
            FisrtArg->setName("staddr");
            SecondArg->setName("objcSuper");
            ThirdArg->setName("sel");
            st_addr_v = FisrtArg;
            super_v = SecondArg;
            sel_v = ThirdArg;
        }
    }

    // Function body

    // check self is not null
    if (isObjcMsgSend()) {
        auto SelfIsNull = new ICmpInst(*entryBlock, CmpInst::ICMP_EQ, self_v, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), self_v->getType()), "self_is_null");
        BranchInst::Create(returnBlock, cacheBlock, SelfIsNull, entryBlock);
    } else {
        BranchInst::Create(cacheBlock, entryBlock);
    }

    // compiler is crashed by GetElementPtrInst::getGEPReturnType which is called in the constructor of the named class.

    // get class
    Value *Cls;
    if (isObjcMsgSend()) {
        auto SelfIsa = new BitCastInst(self_v, PointerType::getUnqual(self_v->getType()), "self.isa", cacheBlock);
        Cls = new LoadInst(SelfIsa->getDestTy(), SelfIsa, "cls", cacheBlock);
    } *//*else if (isObjcMsgSendSuper()) {
        auto ObjcSuperSelf = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index00, "objcSuper.self", cacheBlock);
        self_v = new LoadInst(ObjcSuperSelf->getResultElementType(), ObjcSuperSelf, "self", cacheBlock);

        auto ObjcSuperCls = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index01, "objcSuper.cls", cacheBlock);
        Cls = new LoadInst(ObjcSuperCls->getResultElementType(), ObjcSuperCls, "supercls", cacheBlock);
    } else if (isObjcMsgSendSuper2()) {
        auto ObjcSuperSelf = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index00, "objcSuper.self", cacheBlock);
        self_v = new LoadInst(ObjcSuperSelf->getResultElementType(), ObjcSuperSelf, "self", cacheBlock);

        auto ObjcSuperCls = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index01, "objcSuper.cls", cacheBlock);
        auto MyClsAddr = new BitCastInst(ObjcSuperCls, PointerType::getUnqual(ObjcSuperCls->getType()), "cls_addr", cacheBlock);
        auto MyCls = new LoadInst(MyClsAddr->getDestTy(), MyClsAddr, "cls", cacheBlock);

        auto SuperClsAddr = GetElementPtrInst::CreateInBounds(MyCls->getType(), MyCls, Index1, "cls.super", cacheBlock);
        Cls = new LoadInst(SuperClsAddr->getResultElementType(), SuperClsAddr, "supercls", cacheBlock);
    }*//* else {
        llvm_unreachable("Unexpected function type");
    }

    // call cache_getImp
    FunctionCallee CacheGetImpFunc = getCacheGetImpFunction(module);
    Value *CacheGetImpArgs[] = { Cls, sel_v };
    auto CacheImp = CallInst::Create(CacheGetImpFunc, CacheGetImpArgs, "cache_imp", cacheBlock);
    auto CacheImpIsNull = new ICmpInst(*cacheBlock, CmpInst::ICMP_EQ, CacheImp, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), CacheImp->getType()), "cache_imp_is_null");
    BranchInst::Create(lookupBlock, checkBlock, CacheImpIsNull, cacheBlock);

    // call _class_lookupMethodAndLoadCache3
    FunctionCallee LookupFunc = getLookupMethodAndLoadCache3Functions(module);
    Value *LookupArgs[] = { self_v, sel_v, Cls };
    auto LookupImp = CallInst::Create(LookupFunc, LookupArgs, "lookup_imp", lookupBlock);
    BranchInst::Create(checkBlock, lookupBlock);

    // check imp is not negative
    auto Imp = PHINode::Create(CacheImp->getType(), 2, "imp", checkBlock);
    Imp->addIncoming(CacheImp, cacheBlock);
    Imp->addIncoming(LookupImp, lookupBlock);
    auto ImpIsZeroOrPositive = new ICmpInst(*checkBlock, CmpInst::ICMP_SGE, Imp, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), Imp->getType()), "imp_ge_zero");
    BranchInst::Create(callBlock, forwardBlock, ImpIsZeroOrPositive, checkBlock);

    // call actual function
    SmallVector<Value*, 10> ActualArgs(arguments);
    if (!isObjcMsgSend()) {
        ActualArgs[!isStret() ? 0 : 1] = self_v;
    }
    //auto ActualFunc = new BitCastInst(Imp, getInvokedFunctionType(), "actual_func", callBlock);
    //llvm::CallInst *CallRet = CallInst::Create(ActualFunc, ActualArgs, isVoidReturn ? "" : "call_ret", callBlock);
    llvm::CallInst *CallRet = CallInst::Create(this->getInvokedFunctionType2(), Imp, ActualArgs, isVoidReturn ? "" : "call_ret", callBlock);
    BranchInst::Create(returnBlock, callBlock);

    // forward to method missing
    // not stret: margs = args, return_storage = margs
    // stret: margs = args.slice(1), return_storage = staddr(= args[0])
    auto arg_size = ConstantInt::get(ir_i32_t, isStret() ? ActualArgs.size()-1 : ActualArgs.size());
    auto Margs = new AllocaInst(self_v->getType(), 0, arg_size, "margs", forwardBlock);
    Value *ReturnStorage;
    auto ForwardArgBegin = ActualArgs.begin();
    if (!isStret()) {
        ReturnStorage = GetElementPtrInst::CreateInBounds(Margs->getType(), Margs, Index00, "return_storage", forwardBlock);
    } else {
        ReturnStorage = st_addr_v;
        ForwardArgBegin++;
    }
    unsigned i = 0;
    for(Value *Arg: SmallVector<Value*,0>(ForwardArgBegin, ActualArgs.end())) {
        Value *Indexes[] = {ConstantInt::get(ir_i32_t, 0), ConstantInt::get(ir_i32_t, i)};
        Value* Ptr = GetElementPtrInst::CreateInBounds(Margs->getType(), Margs, Indexes, "ptr", forwardBlock);
        auto ArgPointerType = PointerType::getUnqual(Arg->getType());
        if (ArgPointerType != Ptr->getType()) {
            Ptr = new BitCastInst(Ptr, ArgPointerType, "", forwardBlock);
        }
        new StoreInst(Arg, Ptr, forwardBlock);
        i++;
    }
    FunctionCallee ForwardingFunc = getForwardingFunction(module);
    Value *ForwardingArgs[] = {Margs, ReturnStorage};
    //Type *ForwardingArgsType[] = {Margs->getType(), ReturnStorage->getType()};
    //auto ForwardingFuncType = FunctionType::get(ForwardingFunc.getFunctionType()->getReturnType(), ForwardingArgsType, false);
    CallInst* Target = CallInst::Create(ForwardingFunc, ForwardingArgs, "target", forwardBlock);
    //auto Target = CallInst::Create(ConstantExpr::getBitCast(ForwardingFunc, ForwardingFuncType), ForwardingArgs, "target", forwardBlock);
    auto TargetIsNull = new ICmpInst(*forwardBlock, CmpInst::ICMP_EQ, Target, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), Target->getType()), "target_is_null");
    BranchInst::Create(isVoidReturn ? returnBlock : forwardReturnBlock, retargetBlock, TargetIsNull, forwardBlock);

    // Call objc_msgSend with setting target as self
    // TODO implement
    auto AbortFunc = getAbortFunction(module);
    CallInst::Create(AbortFunc, "", retargetBlock);
    new UnreachableInst(Func->getContext(), retargetBlock);

    if (!isVoidReturn) {
        // return forward result
        auto ForwardRetAddr = new BitCastInst(ReturnStorage, PointerType::getUnqual(return_t), "forward_ret_addr", forwardReturnBlock);
        auto ForwardRet = new LoadInst(ForwardRetAddr->getDestTy(), ForwardRetAddr, "forward_ret", forwardReturnBlock);
        BranchInst::Create(returnBlock, forwardReturnBlock);

        // return
        auto Ret = PHINode::Create(return_t, 3, "ret", returnBlock);


        if (isObjcMsgSend()) {
            auto ZeroValue = ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), return_t);
            Ret->addIncoming(ZeroValue, entryBlock);
        }
        Ret->addIncoming(CallRet, callBlock);
        Ret->addIncoming(ForwardRet, forwardReturnBlock);
        ReturnInst::Create(Func->getContext(), Ret, returnBlock);
    } else {
        ReturnInst::Create(Func->getContext(), returnBlock);
    }
    LLVM_DEBUG(Func->dump());*/
    return Func;
}

// Generates IR instructions
llvm::Function *ObjcFunction::generateMethodInvokeFunction_objc_GS_ABI2(llvm::Module &module)
{
    //printf("generateMethodInvokeFunction_objc_GS_ABI2: %s\n", this->getFullName().c_str());
    using namespace llvm;
    Function* Func = Function::Create(m_messageFunctionType, GlobalValue::InternalLinkage, getFullName(), &module);

    bool isVoidReturn = m_messageFunctionType->getReturnType()->isVoidTy();

    BasicBlock *BB = BasicBlock::Create(Func->getContext(), "", Func);

    SmallVector<Value*, 10> arguments;
    for(auto &Arg: Func->args()) {
        Arg.setName("arg");
        arguments.push_back(&Arg);
    }
    auto ArgIter = Func->arg_begin();
    Argument *FisrtArg = &*ArgIter++;
    Argument *SecondArg = &*ArgIter++;
    Argument *ThirdArg = isStret() ? &*ArgIter : nullptr;

    Value *Method;
    if (!isStret()) {
        FisrtArg->setName("self");
        SecondArg->setName("method");
        Method = SecondArg;
    } else {
        FisrtArg->setName("staddr");
        SecondArg->setName("self");
        ThirdArg->setName("method");
        Method = ThirdArg;
    }
    Type *i8Type = Type::getInt8PtrTy(Func->getContext());
    auto Meth = new BitCastInst(Method, PointerType::getUnqual(i8Type), "method.sel", BB);
    auto sel_v = new LoadInst(Meth->getDestTy(), Meth, "sel", BB);
    assert(sel_v);

    Type *ir_i32_t = Type::getInt32Ty(Func->getContext());
    Value *Index2[] = {ConstantInt::get(ir_i32_t, 2)};
    auto ImpAddr = GetElementPtrInst::CreateInBounds(Meth->getDestTy(), Meth, Index2, "method.imp", BB);
    auto Imp = new LoadInst(ImpAddr->getType(), ImpAddr, "imp", BB);
    assert(Imp);

    SmallVector<Value*, 10> InvokeArgs(arguments);
    InvokeArgs[!isStret() ? 1 : 2] = sel_v;
    //auto InvokedFunc = new BitCastInst(Imp, getInvokedFunctionType(), "func", BB);
    auto Ret = CallInst::Create(this->getInvokedFunctionType2(), Imp, InvokeArgs, isVoidReturn ? "" : "ret", BB);
    if (!isVoidReturn) {
        ReturnInst::Create(Func->getContext(), Ret, BB);
    } else {
        ReturnInst::Create(Func->getContext(), BB);
    }
    LLVM_DEBUG(Func->dump());
    return Func;
}


// mark: Apple Objc4

// Generates IR instructions
llvm::Function *ObjcFunction::generateMsgSendFunction_objc4(llvm::Module &module)
{
    //printf("generateMsgSendFunction_objc4: %s\n", this->getFullName().c_str());
    using namespace llvm;
    Function* Func = Function::Create(m_messageFunctionType, GlobalValue::InternalLinkage, getFullName(), &module);

    // common Type
    Type *ir_i32_t = Type::getInt32Ty(Func->getContext());
    Type *return_t = m_messageFunctionType->getReturnType();
    Value *Index1[] = {ConstantInt::get(ir_i32_t, 1)};
    Value *Index00[] = {ConstantInt::get(ir_i32_t, 0), ConstantInt::get(ir_i32_t, 0)};
    Value *Index01[] = {ConstantInt::get(ir_i32_t, 0), ConstantInt::get(ir_i32_t, 1)};
    bool isVoidReturn = return_t->isVoidTy();

    // Prepare blocks
    BasicBlock *entryBlock = BasicBlock::Create(Func->getContext(), "", Func);
    BasicBlock *cacheBlock = BasicBlock::Create(Func->getContext(), "Cache", Func);
    BasicBlock *lookupBlock = BasicBlock::Create(Func->getContext(), "Lookup", Func);
    BasicBlock *checkBlock = BasicBlock::Create(Func->getContext(), "Check", Func);
    BasicBlock *callBlock = BasicBlock::Create(Func->getContext(), "Call", Func);
    BasicBlock *forwardBlock = BasicBlock::Create(Func->getContext(), "Foward", Func);
    BasicBlock *retargetBlock = BasicBlock::Create(Func->getContext(), "Retarget", Func);
    BasicBlock *forwardReturnBlock = isVoidReturn ? nullptr : BasicBlock::Create(Func->getContext(), "ForwardReturn", Func);
    BasicBlock *returnBlock = BasicBlock::Create(Func->getContext(), "Return", Func);


    // Setup arguments
    SmallVector<Value*, 10> arguments;
    for(auto &Arg: Func->args()) {
        Arg.setName("arg");
        arguments.push_back(&Arg);
    }

    auto ArgIter = Func->arg_begin();
    Argument *FisrtArg = &*ArgIter++;
    Argument *SecondArg = &*ArgIter++;
    Argument *ThirdArg = isStret() ? &*ArgIter : nullptr;

    Value *self_v;
    Value *super_v;
    Value *st_addr_v;
    Value *sel_v;

    if (isObjcMsgSend()) {
        if (!isStret()) {
            FisrtArg->setName("self");
            SecondArg->setName("sel");
            self_v = FisrtArg;
            sel_v = SecondArg;
        } else {
            FisrtArg->setName("staddr");
            SecondArg->setName("self");
            ThirdArg->setName("sel");
            st_addr_v = FisrtArg;
            self_v = SecondArg;
            sel_v = ThirdArg;
        }
    } else { // Super or Super2
        if (!isStret()) {
            FisrtArg->setName("objcSuper");
            SecondArg->setName("sel");
            super_v = FisrtArg;
            sel_v = SecondArg;
        } else {
            FisrtArg->setName("staddr");
            SecondArg->setName("objcSuper");
            ThirdArg->setName("sel");
            st_addr_v = FisrtArg;
            super_v = SecondArg;
            sel_v = ThirdArg;
        }
    }

    // Function body

    // check self is not null
    if (isObjcMsgSend()) {
        auto SelfIsNull = new ICmpInst(*entryBlock, CmpInst::ICMP_EQ, self_v, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), self_v->getType()), "self_is_null");
        BranchInst::Create(returnBlock, cacheBlock, SelfIsNull, entryBlock);
    } else {
        BranchInst::Create(cacheBlock, entryBlock);
    }

    // get class
    Value *Cls;
    if (isObjcMsgSend()) {
        auto SelfIsa = new BitCastInst(self_v, PointerType::getUnqual(self_v->getType()), "self.isa", cacheBlock);
        Cls = new LoadInst(SelfIsa->getDestTy(), SelfIsa, "cls", cacheBlock);
    } else if (isObjcMsgSendSuper()) {
        auto ObjcSuperSelf = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index00, "objcSuper.self", cacheBlock);
        self_v = new LoadInst(ObjcSuperSelf->getResultElementType(), ObjcSuperSelf, "self", cacheBlock);

        auto ObjcSuperCls = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index01, "objcSuper.cls", cacheBlock);
        Cls = new LoadInst(ObjcSuperCls->getResultElementType(), ObjcSuperCls, "supercls", cacheBlock);
    } else if (isObjcMsgSendSuper2()) {
        auto ObjcSuperSelf = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index00, "objcSuper.self", cacheBlock);
        self_v = new LoadInst(ObjcSuperSelf->getResultElementType(), ObjcSuperSelf, "self", cacheBlock);

        auto ObjcSuperCls = GetElementPtrInst::CreateInBounds(super_v->getType(), super_v, Index01, "objcSuper.cls", cacheBlock);
        auto MyClsAddr = new BitCastInst(ObjcSuperCls, PointerType::getUnqual(ObjcSuperCls->getType()), "cls_addr", cacheBlock);
        auto MyCls = new LoadInst(MyClsAddr->getDestTy(), MyClsAddr, "cls", cacheBlock);

        auto SuperClsAddr = GetElementPtrInst::CreateInBounds(MyCls->getType(), MyCls, Index1, "cls.super", cacheBlock);
        Cls = new LoadInst(SuperClsAddr->getResultElementType(), SuperClsAddr, "supercls", cacheBlock);
    } else {
        llvm_unreachable("Unexpected function type");
    }

    // call cache_getImp
    FunctionCallee CacheGetImpFunc = getCacheGetImpFunction(module);
    Value *CacheGetImpArgs[] = { Cls, sel_v };
    auto CacheImp = CallInst::Create(CacheGetImpFunc, CacheGetImpArgs, "cache_imp", cacheBlock);
    auto CacheImpIsNull = new ICmpInst(*cacheBlock, CmpInst::ICMP_EQ, CacheImp, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), CacheImp->getType()), "cache_imp_is_null");
    BranchInst::Create(lookupBlock, checkBlock, CacheImpIsNull, cacheBlock);

    // call _class_lookupMethodAndLoadCache3
    FunctionCallee LookupFunc = getLookupMethodAndLoadCache3Functions(module);
    Value *LookupArgs[] = { self_v, sel_v, Cls };
    auto LookupImp = CallInst::Create(LookupFunc, LookupArgs, "lookup_imp", lookupBlock);
    BranchInst::Create(checkBlock, lookupBlock);

    // check imp is not negative
    auto Imp = PHINode::Create(CacheImp->getType(), 2, "imp", checkBlock);
    Imp->addIncoming(CacheImp, cacheBlock);
    Imp->addIncoming(LookupImp, lookupBlock);
    auto ImpIsZeroOrPositive = new ICmpInst(*checkBlock, CmpInst::ICMP_SGE, Imp, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), Imp->getType()), "imp_ge_zero");
    BranchInst::Create(callBlock, forwardBlock, ImpIsZeroOrPositive, checkBlock);

    // call actual function
    SmallVector<Value*, 10> ActualArgs(arguments);
    if (!isObjcMsgSend()) {
        ActualArgs[!isStret() ? 0 : 1] = self_v;
    }
    //auto ActualFunc = new BitCastInst(Imp, getInvokedFunctionType(), "actual_func", callBlock);
    //llvm::CallInst *CallRet = CallInst::Create(ActualFunc, ActualArgs, isVoidReturn ? "" : "call_ret", callBlock);
    llvm::CallInst *CallRet = CallInst::Create(this->getInvokedFunctionType2(), Imp, ActualArgs, isVoidReturn ? "" : "call_ret", callBlock);
    BranchInst::Create(returnBlock, callBlock);

    // forward to method missing
    // not stret: margs = args, return_storage = margs
    // stret: margs = args.slice(1), return_storage = staddr(= args[0])
    auto arg_size = ConstantInt::get(ir_i32_t, isStret() ? ActualArgs.size()-1 : ActualArgs.size());
    auto Margs = new AllocaInst(self_v->getType(), 0, arg_size, "margs", forwardBlock);
    Value *ReturnStorage;
    auto ForwardArgBegin = ActualArgs.begin();
    if (!isStret()) {
        ReturnStorage = GetElementPtrInst::CreateInBounds(Margs->getType(), Margs, Index00, "return_storage", forwardBlock);
    } else {
        ReturnStorage = st_addr_v;
        ForwardArgBegin++;
    }
    unsigned i = 0;
    for(Value *Arg: SmallVector<Value*,0>(ForwardArgBegin, ActualArgs.end())) {
        Value *Indexes[] = {ConstantInt::get(ir_i32_t, 0), ConstantInt::get(ir_i32_t, i)};
        Value* Ptr = GetElementPtrInst::CreateInBounds(Margs->getType(), Margs, Indexes, "ptr", forwardBlock);
        auto ArgPointerType = PointerType::getUnqual(Arg->getType());
        if (ArgPointerType != Ptr->getType()) {
            Ptr = new BitCastInst(Ptr, ArgPointerType, "", forwardBlock);
        }
        new StoreInst(Arg, Ptr, forwardBlock);
        i++;
    }
    FunctionCallee ForwardingFunc = getForwardingFunction(module);
    Value *ForwardingArgs[] = {Margs, ReturnStorage};
    //Type *ForwardingArgsType[] = {Margs->getType(), ReturnStorage->getType()};
    //auto ForwardingFuncType = /*PointerType::getUnqual(*/FunctionType::get(ForwardingFunc.getFunctionType()->getReturnType(), ForwardingArgsType, false);
    CallInst* Target = CallInst::Create(ForwardingFunc, ForwardingArgs, "target", forwardBlock);
    //auto Target = CallInst::Create(ConstantExpr::getBitCast(ForwardingFunc, ForwardingFuncType), ForwardingArgs, "target", forwardBlock);
    auto TargetIsNull = new ICmpInst(*forwardBlock, CmpInst::ICMP_EQ, Target, ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), Target->getType()), "target_is_null");
    BranchInst::Create(isVoidReturn ? returnBlock : forwardReturnBlock, retargetBlock, TargetIsNull, forwardBlock);

    // Call objc_msgSend with setting target as self
    // TODO implement
    auto AbortFunc = getAbortFunction(module);
    CallInst::Create(AbortFunc, "", retargetBlock);
    new UnreachableInst(Func->getContext(), retargetBlock);

    if (!isVoidReturn) {
        // return forward result
        auto ForwardRetAddr = new BitCastInst(ReturnStorage, PointerType::getUnqual(return_t), "forward_ret_addr", forwardReturnBlock);
        auto ForwardRet = new LoadInst(ForwardRetAddr->getDestTy(), ForwardRetAddr, "forward_ret", forwardReturnBlock);
        BranchInst::Create(returnBlock, forwardReturnBlock);

        // return
        auto Ret = PHINode::Create(return_t, 3, "ret", returnBlock);


        if (isObjcMsgSend()) {
            auto ZeroValue = ConstantExpr::getBitCast(ConstantInt::get(ir_i32_t, 0), return_t);
            Ret->addIncoming(ZeroValue, entryBlock);
        }
        Ret->addIncoming(CallRet, callBlock);
        Ret->addIncoming(ForwardRet, forwardReturnBlock);
        ReturnInst::Create(Func->getContext(), Ret, returnBlock);
    } else {
        ReturnInst::Create(Func->getContext(), returnBlock);
    }
    LLVM_DEBUG(Func->dump());
    return Func;
}

// Generates IR instructions
llvm::Function *ObjcFunction::generateMethodInvokeFunction_objc4(llvm::Module &module)
{
    //printf("generateMethodInvokeFunction_objc4: %s\n", this->getFullName().c_str());
    using namespace llvm;
    Function* Func = Function::Create(m_messageFunctionType, GlobalValue::InternalLinkage, getFullName(), &module);

    bool isVoidReturn = m_messageFunctionType->getReturnType()->isVoidTy();

    BasicBlock *BB = BasicBlock::Create(Func->getContext(), "", Func);

    SmallVector<Value*, 10> arguments;
    for(auto &Arg: Func->args()) {
        Arg.setName("arg");
        arguments.push_back(&Arg);
    }
    auto ArgIter = Func->arg_begin();
    Argument *FisrtArg = &*ArgIter++;
    Argument *SecondArg = &*ArgIter++;
    Argument *ThirdArg = isStret() ? &*ArgIter : nullptr;

    Value *Method;
    if (!isStret()) {
        FisrtArg->setName("self");
        SecondArg->setName("method");
        Method = SecondArg;
    } else {
        FisrtArg->setName("staddr");
        SecondArg->setName("self");
        ThirdArg->setName("method");
        Method = ThirdArg;
    }
    Type *i8Type = Type::getInt8PtrTy(Func->getContext());
    auto Meth = new BitCastInst(Method, PointerType::getUnqual(i8Type), "method.sel", BB);
    auto sel_v = new LoadInst(Meth->getDestTy(), Meth, "sel", BB);
    assert(sel_v);

    Type *ir_i32_t = Type::getInt32Ty(Func->getContext());
    Value *Index2[] = {ConstantInt::get(ir_i32_t, 2)};
    auto ImpAddr = GetElementPtrInst::CreateInBounds(Meth->getType(), Meth, Index2, "method.imp", BB);
    auto Imp = new LoadInst(ImpAddr->getType(), ImpAddr, "imp", BB);
    assert(Imp);

    SmallVector<Value*, 10> InvokeArgs(arguments);
    InvokeArgs[!isStret() ? 1 : 2] = sel_v;
    //auto InvokedFunc = new BitCastInst(Imp, getInvokedFunctionType(), "func", BB);
    auto Ret = CallInst::Create(this->getInvokedFunctionType2(), Imp, InvokeArgs, isVoidReturn ? "" : "ret", BB);
    if (!isVoidReturn) {
        ReturnInst::Create(Func->getContext(), Ret, BB);
    } else {
        ReturnInst::Create(Func->getContext(), BB);
    }
    LLVM_DEBUG(Func->dump());
    return Func;
}

void ObjcCallVisitor::visitCallInst(CallInst &callInst) {
    const Function *fn = callInst.getCalledFunction();
    if (!fn) {
        auto *op = callInst.getCalledOperand();
        op = op->stripPointerCasts();
        fn = dyn_cast<const Function>(op);
    }
    if (!fn) {
        return;
    }

    if (!ObjcFunction::isObjCMsgCalle(fn)) {
        return;
    }

    ObjcFunction function(fn->getName().str(), callInst.getFunctionType(), callInst.getParent()->getModule());

    funcs[function].push_back(&callInst);
}

void ObjcCallVisitor::visitInvokeInst(InvokeInst &invoke) {
    const Function *fn = invoke.getCalledFunction();
    if(!fn) {
        const Value *op = invoke.getCalledOperand();
        op = op->stripPointerCasts();
        fn = dyn_cast<const Function>(op);
    }
    if(!fn) {
        return;
    }

    if(!ObjcFunction::isObjCMsgCalle(fn)) {
        return;
    }

    ObjcFunction function(fn->getName().str(), invoke.getFunctionType(), invoke.getParent()->getModule());

    funcs[function].push_back(&invoke);
}

// TODO: remove downcast
/*
void ObjcCallVisitor::handleCall(CallBase *CI) {
    const Function *fn = callInst->getCalledFunction();
    if (!fn) {
        CV = CV->stripPointerCasts();
        fn = dyn_cast<const Function>(CV);
    }
    if (!fn) {
        return;
    }

    StringRef name = fn->getName();
    if (!(name.startswith(StringRef(OBJC_MSG_SEND)) || name.startswith(StringRef(OBJC_MSG_SEND_SUPER))  || name.startswith(StringRef(OBJC_MSG_SEND_SUPER2)) || name.startswith(StringRef(OBJC_METHOD_INVOKE)))) {
        return;
    }

    ObjcFunction function(fn->getName(), CS.getFunctionType(), CI->getParent()->getModule());

    funcs[function].push_back(CI);
}*/


ModulePass *llvm::createWebAssemblyObjCMsgSendFuncsPass() {
    return new WebAssemblyGenObjCMsgSendFuncs();
}

#endif
