package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A {@link BoundingBox2D} can be used to define from a set of minimum and maximum coordinates an
 * axis-aligned bounding box in the XY-plane.
 */
public class BoundingBox2D implements BoundingBox2DBasics, Settable<BoundingBox2D>
{
   /** The minimum coordinates of this bounding box. */
   private final Point2D minPoint = new Point2D();
   /** The maximum coordinates of this bounding box. */
   private final Point2D maxPoint = new Point2D();

   /**
    * Creates a new bounding box 2D from its center coordinate {@code center} and a tuple 2D holding
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
   public static BoundingBox2D createUsingCenterAndPlusMinusVector(Point2DReadOnly center, Tuple2DReadOnly plusMinusTuple)
   {
      BoundingBox2D boundingBox2D = new BoundingBox2D();
      boundingBox2D.set(center, new Vector2D(plusMinusTuple));
      return boundingBox2D;
   }

   /**
    * Creates a new bounding box such that it is the smallest bounding box containing the two given
    * bounding boxes {@code boundingBoxOne} and {@code boundingBoxTwo}.
    *
    * @param boundingBoxOne the first bounding box. Not modified.
    * @param boundingBoxTwo the second bounding box. Not modified.
    * @return the new bounding box.
    */
   public static BoundingBox2D union(BoundingBox2DReadOnly boundingBoxOne, BoundingBox2DReadOnly boundingBoxTwo)
   {
      BoundingBox2D union = new BoundingBox2D();
      union.combine(boundingBoxOne, boundingBoxTwo);
      return union;
   }

   /**
    * Creates a new bounding box initialized with both its minimum and maximum coordinates to
    * ({@code Double.NaN}, {@code Double.NaN}).
    */
   public BoundingBox2D()
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
   public BoundingBox2D(Point2DReadOnly min, Point2DReadOnly max)
   {
      set(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to {@code other}.
    *
    * @param other the other bounding box used to initialize this. Not modified.
    */
   public BoundingBox2D(BoundingBox2DReadOnly other)
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
   public BoundingBox2D(double[] min, double[] max)
   {
      set(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param minX the minimum x-coordinate for this.
    * @param minY the minimum y-coordinate for this.
    * @param maxX the maximum x-coordinates for this.
    * @param maxY the maximum y-coordinates for this.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   public BoundingBox2D(double minX, double minY, double maxX, double maxY)
   {
      set(minX, minY, maxX, maxY);
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
   @Override
   public void set(BoundingBox2D other)
   {
      BoundingBox2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Point2D getMinPoint()
   {
      return minPoint;
   }

   /** {@inheritDoc} */
   @Override
   public Point2D getMaxPoint()
   {
      return maxPoint;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof BoundingBox2DReadOnly)
         return equals((EuclidGeometry) object);
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
    * Provides a {@code String} representation of this bounding box 2D as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 )
    * </pre>
    *
    * @return the {@code String} representing this bounding box 2D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
