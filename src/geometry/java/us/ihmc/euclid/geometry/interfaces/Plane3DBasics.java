package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for an infinitely wide and long 3D plane defined by a 3D point and a 3D
 * unit-vector.
 */
public interface Plane3DBasics extends Plane3DReadOnly, Clearable, Transformable
{
   /**
    * Gets the reference to the point through which this plane is going.
    *
    * @return the reference to the point.
    */
   @Override
   Point3DBasics getPoint();

   /**
    * Gets the reference to the normal of this plane.
    *
    * @return the reference to the normal.
    */
   @Override
   Vector3DBasics getNormal();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Plane3DReadOnly.super.containsNaN();
   }

   /**
    * Sets the point and normal of this plane to {@link Double#NaN}. After calling this method, this
    * plane becomes invalid. A new valid point and valid normal will have to be set so this plane is
    * again usable.
    */
   @Override
   default void setToNaN()
   {
      getPoint().setToNaN();
      getNormal().setToNaN();
   }

   /**
    * Sets the point of this plane to zero and its normal to (1.0, 0.0, 0.0).
    */
   @Override
   default void setToZero()
   {
      getPoint().setToZero();
      getNormal().setToZero();
   }

   /**
    * Redefines this plane with a new point and a new normal.
    *
    * @param pointOnPlaneX the new x-coordinate of the point on this plane.
    * @param pointOnPlaneY the new y-coordinate of the point on this plane.
    * @param pointOnPlaneZ the new z-coordinate of the point on this plane.
    * @param planeNormalX  the new x-component of the normal of this plane.
    * @param planeNormalY  the new y-component of the normal of this plane.
    * @param planeNormalZ  the new z-component of the normal of this plane.
    */
   default void set(double pointOnPlaneX, double pointOnPlaneY, double pointOnPlaneZ, double planeNormalX, double planeNormalY, double planeNormalZ)
   {
      getPoint().set(pointOnPlaneX, pointOnPlaneY, pointOnPlaneZ);
      getNormal().set(planeNormalX, planeNormalY, planeNormalZ);
   }

   /**
    * Sets this plane to be the same as the given plane.
    *
    * @param other the other plane to copy. Not modified.
    */
   default void set(Plane3DReadOnly other)
   {
      getPoint().set(other.getPoint());
      getNormal().set(other.getNormal());
   }

   /**
    * Redefines this plane such that it goes through the three given points.
    *
    * @param firstPointOnPlane  first point on this plane. Not modified.
    * @param secondPointOnPlane second point on this plane. Not modified.
    * @param thirdPointOnPlane  second point on this plane. Not modified.
    * @return {@code true} if the method succeeded, {@code false} if the method failed to estimate the
    *         normal.
    */
   default boolean set(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      getPoint().set(firstPointOnPlane);
      return EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, getNormal());
   }

   /**
    * Redefines this plane with a new point and a new normal.
    *
    * @param pointOnPlane new point on this plane. Not modified.
    * @param planeNormal  new normal of this plane. Not modified.
    */
   default void set(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      getPoint().set(pointOnPlane);
      getNormal().set(planeNormal);
   }

   /**
    * Transforms this plane using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on this plane's point and normal. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      getPoint().applyTransform(transform);
      getNormal().applyTransform(transform);
   }

   /**
    * Transforms this plane using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on this plane's point and normal. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      getPoint().applyInverseTransform(transform);
      getNormal().applyInverseTransform(transform);
   }
}
