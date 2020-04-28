package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Provides a {@link ConvexPolytope3DReadOnly} view backed by a {@link Ramp3DReadOnly}.
 * <p>
 * The implementation is expected to always reflect the current state of the ramp and its geometry
 * components are expected to be expressed in the global coordinate system of the ramp, i.e.
 * accounting for the ramp's pose.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface RampPolytope3DView extends ConvexPolytope3DReadOnly
{
   /**
    * Quick access to the face that represents the incline of the ramp.
    * 
    * @return the face corresponding to the incline face.
    */
   Face3DReadOnly getRampFace();

   /**
    * Quick access to the ramp's face which normal is {@code (1, 0, 0)} in the ramp's local coordinate
    * system.
    * 
    * @return the face representing the local maximum x coordinate of the ramp.
    */
   Face3DReadOnly getXMaxFace();

   /**
    * Quick access to the ramp's face which normal is {@code (0, 1, 0)} in the ramp's local coordinate
    * system.
    * 
    * @return the face representing the local maximum y coordinate of the ramp.
    */
   Face3DReadOnly getYMaxFace();

   /**
    * Quick access to the ramp's face which normal is {@code (0, -1, 0)} in the ramp's local coordinate
    * system.
    * 
    * @return the face representing the local minimum y coordinate of the ramp.
    */
   Face3DReadOnly getYMinFace();

   /**
    * Quick access to the box's face which normal is {@code (0, 0, -1)} in the ramp's local coordinate
    * system.
    * 
    * @return the face representing the local minimum z coordinate of the ramp.
    */
   Face3DReadOnly getZMinFace();

   /**
    * Quick access to one of the three following faces: {@link #getXMaxFace()}, {@link #getYMaxFace()},
    * and {@link #getRampFace()}.
    * <p>
    * Note that the ramp face is returned when {@code axis == Axis3D#Z}.
    * </p>
    * 
    * @param axis used to select the adequate face to be returned.
    * @return one the three ramp's face representing one of its local maximum coordinates.
    */
   default Face3DReadOnly getMaxFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getXMaxFace();
         case Y:
            return getYMaxFace();
         case Z:
            return getRampFace();
         default:
            throw new IllegalStateException();
      }
   }

   /**
    * Quick access to one of the three following faces: {@link #getRampFace()}, {@link #getYMinFace()},
    * and {@link #getZMinFace()}.
    * <p>
    * Note that the ramp face is returned when {@code axis == Axis3D#X}.
    * </p>
    * 
    * @param axis used to select the adequate face to be returned.
    * @return one the three ramp's face representing one of its local minimum coordinates.
    */
   default Face3DReadOnly getMinFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getRampFace();
         case Y:
            return getYMinFace();
         case Z:
            return getZMinFace();
         default:
            throw new IllegalStateException();
      }
   }

   /**
    * Gets the ramp this polytope view is backed with.
    * 
    * @return the ramp this polytope view is for.
    */
   Ramp3DReadOnly getOwner();

   /**
    * Returns a deep copy of the owner of this polytope view.
    */
   @Override
   default Ramp3DBasics copy()
   {
      return getOwner().copy();
   }

   /** {@inheritDoc} */
   @Override
   default BoundingBox3DReadOnly getBoundingBox()
   {
      return getOwner().getBoundingBox();
   }

   /** {@inheritDoc} */
   @Override
   default Point3DReadOnly getCentroid()
   {
      return getOwner().getCentroid();
   }

   /** {@inheritDoc} */
   @Override
   default double getVolume()
   {
      return getOwner().getVolume();
   }

   /**
    * Returns {@code 0.0}.
    */
   @Override
   default double getConstructionEpsilon()
   {
      return 0;
   }
}
