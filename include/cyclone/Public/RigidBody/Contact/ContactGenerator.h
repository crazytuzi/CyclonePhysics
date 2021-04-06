#pragma once

#include "Contact.h"

namespace cyclone
{
    /**
    * This is the basic polymorphic interface for contact generators
    * applying to rigid bodies.
    */
    class ContactGenerator
    {
        /**
        * Fills the given contact structure with the generated
        * contact. The contact pointer should point to the first
        * available contact in a contact array, where limit is the
        * maximum number of contacts in the array that can be written
        * to. The method returns the number of contacts that have
        * been written.
        */
        virtual unsigned AddContact(Contact* contact, unsigned limit) const = 0;
    };
}
