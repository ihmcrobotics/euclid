package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Provides a {@link ConvexPolytope3DReadOnly} view backed by a {@link Box3DReadOnly}.
 * <p>
 * The implementation is expected to always reflect the current state of the box and its geometry
 * components are expected to be expressed in the global coordinate system of the box, i.e.
 * accounting for the box's pose.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface BoxPolytope3DView extends ConvexPolytope3DReadOnly
{
   /**
    * Quick access to the box's face which centroid is at {@code (0.5 * box3D.getSizeX(), 0.0, 0.0)} in
    * the box's local coordinate system.
    *
    * @return the face representing the local maximum x coordinate of the box.
    */
   Face3DReadOnly getXMaxFace();

   /**
    * Quick access to the box's face which centroid is at {@code (0.0, 0.5 * box3D.getSizeY(), 0.0)} in
    * the box's local coordinate system.
    *
    * @return the face representing the local maximum y coordinate of the box.
    */
   Face3DReadOnly getYMaxFace();

   /**
    * Quick access to the box's face which centroid is at {@code (0.0, 0.0, 0.5 * box3D.getSizeZ())} in
    * the box's local coordinate system.
    *
    * @return the face representing the local maximum y coordinate of the box.
    */
   Face3DReadOnly getZMaxFace();

   /**
    * Quick access to one of the three following faces: {@link #getXMaxFace()}, {@link #getYMaxFace()},
    * and {@link #getZMaxFace()}.
    *
    * @param axis used to select the adequate face to be returned.
    * @return one the three box's face representing one of its local maximum coordinates.
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
            return getZMaxFace();
         default:
            throw new IllegalStateException();
      }
   }

   /**
    * Quick access to the box's face which centroid is at {@code (-0.5 * box3D.getSizeX(), 0.0, 0.0)}
    * in the box's local coordinate system.
    *
    * @return the face representing the local minimum x coordinate of the box.
    */
   Face3DReadOnly getXMinFace();

   /**
    * Quick access to the box's face which centroid is at {@code (0.0, -0.5 * box3D.getSizeY(), 0.0)}
    * in the box's local coordinate system.
    *
    * @return the face representing the local minimum y coordinate of the box.
    */
   Face3DReadOnly getYMinFace();

   /**
    * Quick access to the box's face which centroid is at {@code (0.0, 0.0, -0.5 * box3D.getSizeZ())}
    * in the box's local coordinate system.
    *
    * @return the face representing the local maximum y coordinate of the box.
    */
   Face3DReadOnly getZMinFace();

   /**
    * Quick access to one of the three following faces: {@link #getXMinFace()}, {@link #getYMinFace()},
    * and {@link #getZMinFace()}.
    *
    * @param axis used to select the adequate face to be returned.
    * @return one the three box's face representing one of its local minimum coordinates.
    */
   default Face3DReadOnly getMinFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getXMinFace();
         case Y:
            return getYMinFace();
         case Z:
            return getZMinFace();
         default:
            throw new IllegalStateException();
      }
   }

   /**
    * Gets the box this polytope view is backed with.
    *
    * @return the box this polytope view is for.
    */
   Box3DReadOnly getOwner();

   /**
    * Returns a deep copy of the owner of this polytope view.
    */
   @Override
   default Box3DBasics copy()
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
