package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * A {@link BoundingBox3D} can be used to defines from a set of minimum and maximum coordinates an
 * axis-aligned bounding box.
 */
public class BoundingBox3D implements BoundingBox3DBasics, Settable<BoundingBox3D>
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
    * @param center         the center coordinate of the new bounding box. Not modified.
    * @param plusMinusTuple tuple representing half of the size of the new bounding box. Not modified.
    * @return the new bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
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
    * Creates a new bounding box initialized with both its minimum and maximum coordinates to
    * ({@code Double.NaN}, {@code Double.NaN}, {@code Double.NaN}).
    */
   public BoundingBox3D()
   {
      setToNaN();
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
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
    *                          coordinate on the same axis.
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
    *                          coordinate on the same axis.
    */
   public BoundingBox3D(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
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
   @Override
   public Point3DBasics getMinPoint()
   {
      return minPoint;
   }

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    *
    * @return the read-only reference to the maximum coordinate.
    */
   @Override
   public Point3DBasics getMaxPoint()
   {
      return maxPoint;
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
    * Calculates and returns a hash code value from the min and max coordinates of this bounding box.
    *
    * @return the hash code value for this bounding box.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
   }

   /**
    * Provides a {@code String} representation of this bounding box 3D as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @return the {@code String} representing this bounding box 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
