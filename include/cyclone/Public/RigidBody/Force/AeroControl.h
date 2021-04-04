#pragma once

#include "Aero.h"

namespace cyclone
{
    /**
    * A force generator with a control aerodynamic surface. This
    * requires three inertia tensors, for the two extremes and
    * 'resting' position of the control surface.  The latter tensor is
    * the one inherited from the base class, the two extremes are
    * defined in this class.
    */
    class AeroControl : public Aero
    {
    public:
        /**
        * Creates a new aerodynamic control surface with the given
        * properties.
        */
        AeroControl(const Matrix& base, const Matrix& min, const Matrix& max, const Vector3& position,
                    const Vector3* windspeed);

        /**
        * Sets the control position of this control. This * should
        range between -1 (in which case the minTensor value is *
        used), through 0 (where the base-class tensor value is used) *
        to +1 (where the maxTensor value is used). Values outside that
        * range give undefined results.
        */
        void SetControl(real value);

        /**
        * Applies the force to the given rigid body.
        */
        void UpdateForce(RigidBody* body, real deltaTime) override;

    private:
        /**
        * Calculates the final aerodynamic tensor for the current
        * control setting.
        */
        Matrix GetTensor();

    protected:
        /**
        * The aerodynamic tensor for the surface, when the control is at
        * its maximum value.
        */
        Matrix maxTensor;

        /**
        * The aerodynamic tensor for the surface, when the control is at
        * its minimum value.
        */
        Matrix minTensor;

        /**
        * The current position of the control for this surface. This
        * should range between -1 (in which case the minTensor value
        * is used), through 0 (where the base-class tensor value is
        * used) to +1 (where the maxTensor value is used).
        */
        real controlSetting;
    };
}
