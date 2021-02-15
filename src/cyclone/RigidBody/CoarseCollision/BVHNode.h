#pragma once

#include "RigidBody/RigidBody.h"

namespace cyclone
{
    /**
    * Stores a potential contact to check later.
    */
    struct PotentialContact
    {
        /**
        * Holds the bodies that might be in contact.
        */
        RigidBody* body[2];
    };

    /**
    * A base class for nodes in a bounding volume hierarchy.
    *
    * This class uses a binary tree to store the bounding volumes.
    */
    template <class BoundingVolumeClass>
    class BVHNode
    {
    public:
        /**
        * Creates a new node in the hierarchy with the given parameters.
        */
        BVHNode(BVHNode* parent, const BoundingVolumeClass& volume, RigidBody* body = nullptr);

        /**
        * Checks if this node is at the bottom of the hierarchy.
        */
        bool IsLeaf() const;

        /**
        * Checks the potential contacts from this node downwards in
        * the hierarchy, writing them to the given array (up to the
        * given limit). Returns the number of potential contacts it
        * found.
        */
        unsigned GetPotentialContacts(PotentialContact* contacts, unsigned limit) const;

        /**
        * Inserts the given rigid body, with the given bounding volume,
        * into the hierarchy. This may involve the creation of
        * further bounding volume nodes.
        */
        void Insert(RigidBody* newBody, const BoundingVolumeClass& newVolume);

        /**
        * Deletes this node, removing it first from the hierarchy, along with its
        * associated rigid body and child nodes. This method deletes the node
        * and all its children (but obviously not the rigid bodies). This
        * also has the effect of deleting the sibling of this node, and
        * changing the parent node so that it contains the data currently
        * in that sibling. Finally it forces the hierarchy above the
        * current node to reconsider its bounding volume.
        */
        ~BVHNode();

    protected:
        /**
        * Checks for overlapping between nodes in the hierarchy. Note
        * that any bounding volume should have an overlaps method implemented
        * that checks for overlapping with another object of its own type.
        */
        bool Overlay(const BVHNode<BoundingVolumeClass>* other) const;

        /**
        * Checks the potential contacts between this node and the given
        * other node, writing them to the given array (up to the
        * given limit). Returns the number of potential contacts it
        * found.
        */
        unsigned GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>* other, PotentialContact* contacts,
                                          unsigned limit) const;

        /**
        * For non-leaf nodes, this method recalculates the bounding volume
        * based on the bounding volumes of its children.
        */
        void RecalculateBoundingVolume(bool recurse = true);

    public:
        /**
        * Holds the child nodes of this node.
        */
        BVHNode* children[2];

        /**
        * Holds a single bounding volume encompassing all the
        * descendents of this node.
        */
        BoundingVolumeClass volume;

        /**
        * Holds the rigid body at this node of the hierarchy.
        * Only leaf nodes can have a rigid body defined (see isLeaf).
        * Note that it is possible to rewrite the algorithms in this
        * class to handle objects at all levels of the hierarchy,
        * but the code provided ignores this vector unless firstChild
        * is NULL.
        */
        RigidBody* body;

        /**
        * Holds the node immediately above us in the tree.
        */
        BVHNode* parent;
    };

    template <class BoundingVolumeClass>
    BVHNode<BoundingVolumeClass
    >::BVHNode(BVHNode* parent, const BoundingVolumeClass& volume, RigidBody* body): volume(volume), body(body),
        parent(parent)
    {
        children[0] = children[1] = nullptr;
    }

    template <class BoundingVolumeClass>
    bool BVHNode<BoundingVolumeClass>::IsLeaf() const
    {
        return body != nullptr;
    }

    template <class BoundingVolumeClass>
    unsigned BVHNode<BoundingVolumeClass>::GetPotentialContacts(PotentialContact* contacts, unsigned limit) const
    {
        // Early out if we don't have the room for contacts, or if we're a leaf node.
        if (IsLeaf() || limit == 0)
        {
            return 0;
        }

        // Get the potential contacts of one of our children with the other
        return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
    }

    template <class BoundingVolumeClass>
    void BVHNode<BoundingVolumeClass>::Insert(RigidBody* newBody, const BoundingVolumeClass& newVolume)
    {
        // If we are a leaf, then the only option is to spawn two
        // new children and place the new body in one.
        if (IsLeaf())
        {
            // Child one is a copy of us.
            children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);

            // Child two holds the new body
            children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

            // And we now loose the body (we're no longer a leaf)
            this->body = nullptr;

            // We need to recalculate our bounding volume
            RecalculateBoundingVolume();
        }
        else
        {
            // Otherwise we need to work out which child gets to keep
            // the inserted body. We give it to whoever would grow the
            // least to incorporate it.
            if (children[0]->volume.GetGrowth(newVolume) < children[1]->volume.GetGrowth(newVolume))
            {
                children[0]->Insert(newBody, newVolume);
            }
            else
            {
                children[1]->Insert(newBody, newVolume);
            }
        }
    }

    template <class BoundingVolumeClass>
    BVHNode<BoundingVolumeClass>::~BVHNode()
    {
        // If we don't have a parent, then we ignore the sibling processing
        if (parent)
        {
            // Find our sibling
            BVHNode<BoundingVolumeClass>* sibling;

            if (parent->children[0] == this)
            {
                sibling = parent->children[1];
            }
            else
            {
                sibling = parent->children[0];
            }

            // Write its data to our parent
            parent->volume = sibling->volume;

            parent->body = sibling->body;

            parent->children[0] = sibling->children[0];

            parent->children[1] = sibling->children[1];

            // Delete the sibling (we blank its parent and
            // children to avoid processing/deleting them)
            sibling->parent = nullptr;

            sibling->body = nullptr;

            sibling->children[0] = nullptr;

            sibling->children[1] = nullptr;

            delete sibling;

            // Recalculate the parent's bounding volume
            parent->RecalculateBoundingVolume();
        }

        // Delete our children (again we remove their
        // parent data so we don't try to process their siblings
        // as they are deleted).
        if (children[0])
        {
            children[0]->parent = nullptr;

            delete children[0];
        }

        if (children[1])
        {
            children[1]->parent = nullptr;

            delete children[1];
        }
    }

    template <class BoundingVolumeClass>
    bool BVHNode<BoundingVolumeClass>::Overlay(const BVHNode<BoundingVolumeClass>* other) const
    {
        return other != nullptr && volume->Overlay(other->volume);
    }

    template <class BoundingVolumeClass>
    unsigned BVHNode<BoundingVolumeClass>::GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>* other,
                                                                    PotentialContact* contacts, unsigned limit) const
    {
        // Early out if we don't overlap or if we have no room
        // to report contacts
        if (other == nullptr || contacts == nullptr || !Overlay(other) || limit == 0)
        {
            return 0;
        }

        // If we're both at leaf nodes, then we have a potential contact
        if (IsLeaf() && other->IsLeaf())
        {
            contacts->body[0] = body;

            contacts->body[1] = other->body;

            return 1;
        }

        // Determine which node to descend into. If either is
        // a leaf, then we descend the other. If both are branches,
        // then we use the one with the largest size.
        if (other->IsLeaf() || !IsLeaf() && volume->Size() >= other->volume->Size())
        {
            // Recurse into ourself
            auto count = children[0]->GetPotentialContactsWith(other, contacts, limit);

            // Check we have enough slots to do the other side too
            if (limit > count)
            {
                return count + children[1]->GetPotentialContactsWith(other, contacts + count, limit - count);
            }
            else
            {
                return count;
            }
        }
        else
        {
            // Recurse into the other node
            auto count = GetPotentialContactsWith(other->children[0], contacts, limit);

            // Check we have enough slots to do the other side too
            if (limit > count)
            {
                return count + GetPotentialContactsWith(other->children[1], contacts + count, limit - count);
            }
            else
            {
                return count;
            }
        }
    }

    template <class BoundingVolumeClass>
    void BVHNode<BoundingVolumeClass>::RecalculateBoundingVolume(bool recurse)
    {
        if (IsLeaf())
        {
            return;
        }

        // Use the bounding volume combining constructor.
        volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);

        // Recurse up the tree
        if (parent != nullptr)
        {
            parent->RecalculateBoundingVolume(true);
        }
    }
}
