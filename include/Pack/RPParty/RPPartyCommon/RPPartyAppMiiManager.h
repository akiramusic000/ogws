#ifndef RP_PARTY_COMMON_APP_MII_MANAGER_H
#define RP_PARTY_COMMON_APP_MII_MANAGER_H

#include <Pack/types_pack.h>

#include <Pack/RPKernel.h>

#include <egg/core.h>

//! @addtogroup rp_party
//! @{

/**
 * @brief Party Pack Mii data manager
 */
class RPPartyAppMiiManager : public RPSysAppMiiManager {
public:
    /**
     * @brief Constructor
     *
     * @param pHeap Heap to use for allocations
     */
    RPPartyAppMiiManager(EGG::Heap* pHeap);

    /**
     * @brief Loads the Mii data resources
     */
    virtual void LoadResource(); // at 0xC

private:
    // . . .
};

//! @}

#endif
