package us.ihmc.euclid.geometry;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DBasics;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
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
    * Gets the direction defining this plane by storing its components in the given argument
    * {@code planeNormalToPack}.
    *
    * @param planeNormalToPack vector in which the components of this plane's normal are stored.
    *                          Modified.
    * @deprecated Use {@code planeNormalToPack.set(this.getNormal())} instead.
    */
   @Deprecated
   public void getNormal(Vector3DBasics planeNormalToPack)
   {
      planeNormalToPack.set(normal);
   }

   /**
    * Returns a copy of this plane's normal.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return a copy of this plane's normal.
    * @deprecated Use {@code new Vector3D(this.getNormal())} instead.
    */
   @Deprecated
   public Vector3D getNormalCopy()
   {
      return new Vector3D(normal);
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
    * Gets the point defining this plane by storing its coordinates in the given argument
    * {@code pointToPack}.
    *
    * @param pointOnPlaneToPack point in which the coordinates of this plane's point are stored.
    *                           Modified.
    * @deprecated Use {@code pointOnPlaneToPack.set(this.getPoint())} instead.
    */
   @Deprecated
   public void getPoint(Point3DBasics pointOnPlaneToPack)
   {
      pointOnPlaneToPack.set(point);
   }

   /**
    * Returns a copy of the point defining this plane.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return a copy of this plane's point.
    * @deprecated Use {@code new Point3D(this.getPoint())} instead.
    */
   @Deprecated
   public Point3D getPointCopy()
   {
      return new Point3D(point);
   }

   /**
    * Changes the normal of this plane by setting it to the normalized value of the given vector.
    *
    * @param normalX the new x-component of the normal of this normal.
    * @param normalY the new y-component of the normal of this normal.
    * @param normalZ the new z-component of the normal of this normal.
    * @deprecated Use {@code this.getNormal().set(normalX, normalY, normalZ)} instead.
    */
   @Deprecated
   public void setNormal(double normalX, double normalY, double normalZ)
   {
      getNormal().set(normalX, normalY, normalZ);
   }

   /**
    * Changes the direction of this plane by setting it to the normalized value of the given vector.
    *
    * @param planeNormal new normal of this plane. Not modified.
    * @deprecated Use {@code this.getNormal().set(planeNormal)} instead.
    */
   @Deprecated
   public void setNormal(Vector3DReadOnly planeNormal)
   {
      getNormal().set(planeNormal);
   }

   /**
    * Changes the point through which this plane has to go.
    *
    * @param pointX the new x-coordinate of the point on this plane.
    * @param pointY the new y-coordinate of the point on this plane.
    * @param pointZ the new z-coordinate of the point on this plane.
    * @deprecated Use {@code this.getPoint().set(pointX, pointY, pointZ)} instead.
    */
   @Deprecated
   public void setPoint(double pointX, double pointY, double pointZ)
   {
      getPoint().set(pointX, pointY, pointZ);
   }

   /**
    * Changes the point through which this plane has to go.
    *
    * @param pointOnPlane new point on this plane. Not modified.
    * @deprecated Use {@code this.getPoint().set(pointOnPlane)} instead.
    */
   @Deprecated
   public void setPoint(Point3DReadOnly pointOnPlane)
   {
      getPoint().set(pointOnPlane);
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

   /**
    * Provides a {@code String} representation of this plane 3D as follows:<br>
    * Plane 3D: point = (x, y, z), normal = (x, y, z)
    *
    * @param format the format to be used.
    * @return the {@code String} representing this plane 3D.
    */
   @Override
   public String toString(String format)
   {
      return EuclidGeometryIOTools.getPlane3DString(format, this);
   }
}
