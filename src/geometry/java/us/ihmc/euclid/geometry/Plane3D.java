package us.ihmc.euclid.geometry;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DBasics;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely wide and long 3D plane defined by a 3D point and a 3D unit-vector.
 */
public class Plane3D implements Plane3DBasics
{
   /** Coordinates of a point located on this plane. */
   private final Point3D point = new Point3D();
   /**
    * Normal of this plane of unit-length. Its direction indicates which side of the plane is
    * considered to be the 'above' part.
    */
   private final UnitVector3D normal = new UnitVector3D(Axis3D.Z);

   /**
    * Default constructor that initializes its {@code point} to zero and {@code normal} to
    * {@link Axis3D#Z}.
    */
   public Plane3D()
   {
   }

   /**
    * Initializes this plane to be passing through the given point, with the vector as the normal.
    *
    * @param pointOnPlaneX the new x-coordinate of the point on this plane.
    * @param pointOnPlaneY the new y-coordinate of the point on this plane.
    * @param pointOnPlaneZ the new z-coordinate of the point on this plane.
    * @param planeNormalX  the new x-component of the normal of this plane.
    * @param planeNormalY  the new y-component of the normal of this plane.
    * @param planeNormalZ  the new z-component of the normal of this plane.
    */
   public Plane3D(double pointOnPlaneX, double pointOnPlaneY, double pointOnPlaneZ, double planeNormalX, double planeNormalY, double planeNormalZ)
   {
      set(pointOnPlaneX, pointOnPlaneY, pointOnPlaneZ, planeNormalX, planeNormalY, planeNormalZ);
   }

   /**
    * Creates a new plane 3D and initializes it to {@code other}.
    *
    * @param other the other plane used to initialize this plane. Not modified.
    */
   public Plane3D(Plane3D other)
   {
      set(other);
   }

   /**
    * Initializes this plane to be passing through the three given points.
    *
    * @param firstPointOnPlane  first point on this plane. Not modified.
    * @param secondPointOnPlane second point on this plane. Not modified.
    * @param thirdPointOnPlane  second point on this plane. Not modified.
    */
   public Plane3D(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      set(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
   }

   /**
    * Initializes this plane to be passing through the given point, with the vector as the normal.
    *
    * @param pointOnPlane point on this plane. Not modified.
    * @param planeNormal  normal of this plane. Not modified.
    */
   public Plane3D(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      set(pointOnPlane, planeNormal);
   }

   /**
    * Gets the read-only reference to the normal of this plane.
    *
    * @return the reference to the normal.
    */
   @Override
   public UnitVector3DBasics getNormal()
   {
      return normal;
   }

   /**
    * Gets the read-only reference to the point through which this plane is going.
    *
    * @return the reference to the point.
    */
   @Override
   public Point3DBasics getPoint()
   {
      return point;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Plane3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Plane3DReadOnly)
         return Plane3DBasics.super.equals((Plane3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(point, normal);
   }

   /**
    * Provides a {@code String} representation of this plane 3D as follows:<br>
    * Plane 3D: point = (x, y, z), normal = (x, y, z)
    *
    * @return the {@code String} representing this plane 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
