package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * A {@link BoundingBox3D} can be used to defines from a set of minimum and maximum coordinates an
 * axis-aligned bounding box.
 */
public class BoundingBox3D implements BoundingBox3DBasics, EpsilonComparable<BoundingBox3D>, Settable<BoundingBox3D>, GeometricallyComparable<BoundingBox3D>
{
   /** The minimum coordinates of this bounding box. */
   private final Point3D minPoint = new Point3D();
   /** The maximum coordinates of this bounding box. */
   private final Point3D maxPoint = new Point3D();

   /**
    * Creates a new bounding box 3D from its center coordinate {@code center} and a tuple 3D holding
    * onto half its size {@code plusMinusTuple}.
    * <p>
    * The minimum and maximum coordinates of the resulting bounding box are calculated as follows:
    * <ul>
    * <li>{@code minPoint = center - plusMinusTuple}
    * <li>{@code maxPoint = center + plusMinusTuple}
    * </ul>
    * </p>
    *
    * @param center the center coordinate of the new bounding box. Not modified.
    * @param plusMinusTuple tuple representing half of the size of the new bounding box. Not modified.
    * @return the new bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public static BoundingBox3D createUsingCenterAndPlusMinusVector(Point3DReadOnly center, Tuple3DReadOnly plusMinusTuple)
   {
      BoundingBox3D boundingBox3D = new BoundingBox3D();
      boundingBox3D.set(center, new Vector3D(plusMinusTuple));
      return boundingBox3D;
   }

   /**
    * Creates a new bounding box such that it is the smallest bounding box containing the two given
    * bounding boxes {@code boundingBoxOne} and {@code boundingBoxTwo}.
    *
    * @param boundingBoxOne the first bounding box. Not modified.
    * @param boundingBoxTwo the second bounding box. Not modified.
    * @return the new bounding box.
    */
   public static BoundingBox3D union(BoundingBox3DReadOnly boundingBoxOne, BoundingBox3DReadOnly boundingBoxTwo)
   {
      BoundingBox3D union = new BoundingBox3D();
      union.combine(boundingBoxOne, boundingBoxTwo);
      return union;
   }

   /**
    * Creates a new bounding box initialized with both its minimum and maximum coordinates to (0, 0).
    */
   public BoundingBox3D()
   {
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public BoundingBox3D(Point3DReadOnly min, Point3DReadOnly max)
   {
      set(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to {@code other}.
    *
    * @param other the other bounding box used to initialize this. Not modified.
    */
   public BoundingBox3D(BoundingBox3DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public BoundingBox3D(double[] min, double[] max)
   {
      set(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param minX the minimum x-coordinate for this.
    * @param minY the minimum y-coordinate for this.
    * @param minZ the minimum z-coordinate for this.
    * @param maxX the maximum x-coordinates for this.
    * @param maxY the maximum y-coordinates for this.
    * @param maxZ the maximum z-coordinates for this.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public BoundingBox3D(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   @Override
   public void set(BoundingBox3D other)
   {
      BoundingBox3DBasics.super.set(other);
   }

   /**
    * Gets the read-only reference to the minimum coordinate of this bounding box.
    *
    * @return the read-only reference to the minimum coordinate.
    */
   public Point3DBasics getMinPoint()
   {
      return minPoint;
   }

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    *
    * @return the read-only reference to the maximum coordinate.
    */
   public Point3DBasics getMaxPoint()
   {
      return maxPoint;
   }

   /**
    * Tests on a per-component basis on the minimum and maximum coordinates if this bounding box is
    * equal to {@code other} with the tolerance {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two bounding boxes are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(BoundingBox3D other, double epsilon)
   {
      return BoundingBox3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(BoundingBox3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof BoundingBox3DReadOnly)
         return equals((BoundingBox3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this bounding box 3D as follows:<br>
    * Bounding Box 3D: min = (x, y, z), max = (x, y, z)
    *
    * @return the {@code String} representing this bounding box 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getBoundingBox3DString(this);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two bounding boxes are geometrically
    * similar, i.e. the distance between their min and max points is less than or equal to
    * {@code epsilon}.
    *
    * @param other the bounding box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two bounding boxes represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(BoundingBox3D other, double epsilon)
   {
      return BoundingBox3DBasics.super.geometricallyEquals(other, epsilon);
   }
}
