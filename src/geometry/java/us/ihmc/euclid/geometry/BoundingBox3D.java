package us.ihmc.euclid.geometry;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D;

import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * A {@link BoundingBox3D} can be used to defines from a set of minimum and maximum coordinates an
 * axis-aligned bounding box.
 */
public class BoundingBox3D implements EpsilonComparable<BoundingBox3D>, Settable<BoundingBox3D>, Clearable, GeometricallyComparable<BoundingBox3D>
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
   public static BoundingBox3D union(BoundingBox3D boundingBoxOne, BoundingBox3D boundingBoxTwo)
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
   public BoundingBox3D(BoundingBox3D other)
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

   /**
    * Asserts that the minimum coordinates are less or equal to the maximum coordinates.
    *
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void checkBounds()
   {
      if (minPoint.getX() > maxPoint.getX())
         throw new BoundingBoxException("minPoint.getX() > maxPoint.getX(): " + minPoint.getX() + ">" + maxPoint.getX());
      if (minPoint.getY() > maxPoint.getY())
         throw new BoundingBoxException("minPoint.getY() > maxPoint.getY(): " + minPoint.getY() + ">" + maxPoint.getY());
      if (minPoint.getZ() > maxPoint.getZ())
         throw new BoundingBoxException("minPoint.getZ() > maxPoint.getZ(): " + minPoint.getZ() + ">" + maxPoint.getZ());
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMin(Point3DReadOnly min)
   {
      minPoint.set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMin(double[] min)
   {
      minPoint.set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @param minZ the new minimum z-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMin(double minX, double minY, double minZ)
   {
      minPoint.set(minX, minY, minZ);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMax(Point3DReadOnly max)
   {
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMax(double[] max)
   {
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param maxX the new maximum x-coordinate for this bounding box.
    * @param maxY the new maximum y-coordinate for this bounding box.
    * @param maxZ the new maximum z-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMax(double maxX, double maxY, double maxZ)
   {
      maxPoint.set(maxX, maxY, maxZ);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @param minZ the new minimum z-coordinate for this bounding box.
    * @param maxX the new maximum x-coordinates for this bounding box.
    * @param maxY the new maximum y-coordinates for this bounding box.
    * @param maxZ the new maximum z-coordinates for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void set(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      minPoint.set(minX, minY, minZ);
      maxPoint.set(maxX, maxY, maxZ);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void set(double[] min, double[] max)
   {
      minPoint.set(min);
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void set(Point3DReadOnly min, Point3DReadOnly max)
   {
      minPoint.set(min);
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param center the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    */
   public void set(Point3DReadOnly center, Vector3DReadOnly halfSize)
   {
      minPoint.sub(center, halfSize);
      maxPoint.add(center, halfSize);
      checkBounds();
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
   @Override
   public void set(BoundingBox3D other)
   {
      minPoint.set(other.minPoint);
      maxPoint.set(other.maxPoint);
   }

   /**
    * Combines this bounding box with {@code other} such that it becomes the smallest bounding box
    * containing this and {@code other}.
    *
    * @param other the other bounding box to combine with this. Not modified.
    */
   public void combine(BoundingBox3D other)
   {
      combine(this, other);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    */
   public void combine(BoundingBox3D boundingBoxOne, BoundingBox3D boundingBoxTwo)
   {
      double minX = Math.min(boundingBoxOne.getMinX(), boundingBoxTwo.getMinX());
      double minY = Math.min(boundingBoxOne.getMinY(), boundingBoxTwo.getMinY());
      double minZ = Math.min(boundingBoxOne.getMinZ(), boundingBoxTwo.getMinZ());

      double maxX = Math.max(boundingBoxOne.getMaxX(), boundingBoxTwo.getMaxX());
      double maxY = Math.max(boundingBoxOne.getMaxY(), boundingBoxTwo.getMaxY());
      double maxZ = Math.max(boundingBoxOne.getMaxZ(), boundingBoxTwo.getMaxZ());

      set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   /**
    * Invalidates this bounding box by setting all its coordinates to {@link Double#NaN}.
    */
   @Override
   public void setToNaN()
   {
      minPoint.setToNaN();
      maxPoint.setToNaN();
   }

   /**
    * Resets this bounding box by setting both its minimum and maximum coordinates to (0, 0).
    */
   @Override
   public void setToZero()
   {
      minPoint.setToZero();
      maxPoint.setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return minPoint.containsNaN() || maxPoint.containsNaN();
   }

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    *
    * @param centerToPack point 3D in which the center of this bounding box is stored. Modified.
    */
   public void getCenterPoint(Point3DBasics centerToPack)
   {
      centerToPack.interpolate(minPoint, maxPoint, 0.5);
   }

   /**
    * Performs a linear interpolation between this bounding box minimum and maximum coordinates using
    * independent parameters {@code xParameter}, {@code yParameter}, and {@code zParameter} for the
    * x-axis, y-axis, and z-axis respectively. The result is stored in {@code pointToPack}.
    * <p>
    * <ul>
    * <li>{@code (xParameter == 0)} results in: {@code (pointToPack.getX() == this.getMinX())}.
    * <li>{@code (xParameter == 1)} results in: {@code (pointToPack.getX() == this.getMaxX())}.
    * <li>{@code (yParameter == 0)} results in: {@code (pointToPack.getY() == this.getMinY())}.
    * <li>{@code (yParameter == 1)} results in: {@code (pointToPack.getY() == this.getMaxY())}.
    * <li>{@code (zParameter == 0)} results in: {@code (pointToPack.getZ() == this.getMinZ())}.
    * <li>{@code (zParameter == 1)} results in: {@code (pointToPack.getZ() == this.getMaxZ())}.
    * </ul>
    * </p>
    *
    * @param xParameter the parameter to use for the interpolation along the x-axis.
    * @param yParameter the parameter to use for the interpolation along the y-axis.
    * @param zParameter the parameter to use for the interpolation along the z-axis.
    * @param pointToPack the point 3D in which the result is stored. Modified.
    */
   public void getPointGivenParameters(double xParameter, double yParameter, double zParameter, Point3DBasics pointToPack)
   {
      pointToPack.setX(getMinX() + xParameter * (getMaxX() - getMinX()));
      pointToPack.setY(getMinY() + yParameter * (getMaxY() - getMinY()));
      pointToPack.setZ(getMinZ() + zParameter * (getMaxZ() - getMinZ()));
   }

   /**
    * Calculates the squared value of the distance between the minimum and maximum coordinates of this
    * bounding box.
    *
    * @return the squared value of this bounding box diagonal.
    */
   public double getDiagonalLengthSquared()
   {
      return minPoint.distanceSquared(maxPoint);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    */
   public boolean isInsideExclusive(Point3DReadOnly query)
   {
      return isInsideExclusive(query.getX(), query.getY(), query.getZ());
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param x the x-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param y the y-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param z the z-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    */
   public boolean isInsideExclusive(double x, double y, double z)
   {
      if (x <= getMinX() || x >= getMaxX())
         return false;

      if (y <= getMinY() || y >= getMaxY())
         return false;

      if (z <= getMinZ() || z >= getMaxZ())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    */
   public boolean isInsideInclusive(Point3DReadOnly query)
   {
      return isInsideInclusive(query.getX(), query.getY(), query.getZ());
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param x the x-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param y the y-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param z the z-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    */
   public boolean isInsideInclusive(double x, double y, double z)
   {
      if (x < getMinX() || x > getMaxX())
         return false;

      if (y < getMinY() || y > getMaxY())
         return false;

      if (z < getMinZ() || z > getMaxZ())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   public boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param x the x-coordinate the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param y the y-coordinate the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param z the z-coordinate the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   public boolean isInsideEpsilon(double x, double y, double z, double epsilon)
   {
      if (x <= getMinX() - epsilon || x >= getMaxX() + epsilon)
         return false;

      if (y <= getMinY() - epsilon || y >= getMaxY() + epsilon)
         return false;

      if (z <= getMinZ() - epsilon || z >= getMaxZ() + epsilon)
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    */
   public boolean isXYInsideExclusive(Point2DReadOnly query)
   {
      return isXYInsideExclusive(query.getX(), query.getY());
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param x the x-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param y the y-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    */
   public boolean isXYInsideExclusive(double x, double y)
   {
      if (x <= getMinX() || x >= getMaxX())
         return false;

      if (y <= getMinY() || y >= getMaxY())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    */
   public boolean isXYInsideInclusive(Point2DReadOnly query)
   {
      return isXYInsideInclusive(query.getX(), query.getY());
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param x the x-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param y the y-coordinate of the query to test if it is located inside this bounding box. Not
    *           modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    */
   public boolean isXYInsideInclusive(double x, double y)
   {
      if (x < getMinX() || x > getMaxX())
         return false;

      if (y < getMinY() || y > getMaxY())
         return false;

      return true;
   }

   /**
    * * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   public boolean isXYInsideEpsilon(Point2DReadOnly query, double epsilon)
   {
      return isXYInsideEpsilon(query.getX(), query.getY(), epsilon);
   }

   /**
    * * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param x the x-coordinate the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param y the y-coordinate the query to test if it is located inside this bounding box. Not
    *           modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   public boolean isXYInsideEpsilon(double x, double y, double epsilon)
   {
      if (x <= getMinX() - epsilon || x >= getMaxX() + epsilon)
         return false;

      if (y <= getMinY() - epsilon || y >= getMaxY() + epsilon)
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to not be intersecting if they share a corner or an edge.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsExclusive(BoundingBox3D other)
   {
      if (other.getMinX() >= getMaxX() || other.getMaxX() <= getMinX())
         return false;

      if (other.getMinY() >= getMaxY() || other.getMaxY() <= getMinY())
         return false;

      if (other.getMinZ() >= getMaxZ() || other.getMaxZ() <= getMinZ())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to be intersecting if they share a corner or an edge.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsInclusive(BoundingBox3D other)
   {
      if (other.getMinX() > getMaxX() || other.getMaxX() < getMinX())
         return false;

      if (other.getMinY() > getMaxY() || other.getMaxY() < getMinY())
         return false;

      if (other.getMinZ() > getMaxZ() || other.getMaxZ() < getMinZ())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusive(BoundingBox3D)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @param epsilon the tolerance to use in this test.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsEpsilon(BoundingBox3D other, double epsilon)
   {
      if (other.getMinX() >= getMaxX() + epsilon || other.getMaxX() <= getMinX() - epsilon)
         return false;

      if (other.getMinY() >= getMaxY() + epsilon || other.getMaxY() <= getMinY() - epsilon)
         return false;

      if (other.getMinZ() >= getMaxZ() + epsilon || other.getMaxZ() <= getMinZ() - epsilon)
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to not be intersecting if they share a corner or an edge.
    * </p>
    * <p>
    * This method is equivalent to projecting this bounding box onto the XY-plane before performing the
    * test.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsExclusiveInXYPlane(BoundingBox2DReadOnly other)
   {
      if (other.getMinX() >= getMaxX() || other.getMaxX() <= getMinX())
         return false;

      if (other.getMinY() >= getMaxY() || other.getMaxY() <= getMinY())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to be intersecting if they share a corner or an edge.
    * </p>
    * <p>
    * This method is equivalent to projecting this bounding box onto the XY-plane before performing the
    * test.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsInclusiveInXYPlane(BoundingBox2DReadOnly other)
   {
      if (other.getMinX() > getMaxX() || other.getMaxX() < getMinX())
         return false;

      if (other.getMinY() > getMaxY() || other.getMaxY() < getMinY())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusiveInXYPlane(BoundingBox2DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    * <p>
    * This method is equivalent to projecting this bounding box onto the XY-plane before performing the
    * test.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @param epsilon the tolerance to use in this test.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsEpsilonInXYPlane(BoundingBox2DReadOnly other, double epsilon)
   {
      if (other.getMinX() >= getMaxX() + epsilon || other.getMaxX() <= getMinX() - epsilon)
         return false;

      if (other.getMinY() >= getMaxY() + epsilon || other.getMaxY() <= getMinY() - epsilon)
         return false;

      return true;
   }

   /**
    * Tests if this the given line 3D intersects this bounding box.
    *
    * @param line3D the query. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    */
   public boolean doesIntersectWithLine3D(Line3DReadOnly line3D)
   {
      return doesIntersectWithLine3D(line3D.getPoint(), line3D.getDirection());
   }

   /**
    * Tests if this the given line 3D intersects this bounding box.
    *
    * @param pointOnLine a point located on the infinitely long line. Not modified.
    * @param lineDirection the line direction. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    */
   public boolean doesIntersectWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return intersectionWithLine3D(pointOnLine, lineDirection, null, null) > 0;
   }

   /**
    * Tests if this the given line segment 3D intersects this bounding box.
    *
    * @param lineSegment3D the query. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    */
   public boolean doesIntersectWithLineSegment3D(LineSegment3DReadOnly lineSegment3D)
   {
      return doesIntersectWithLineSegment3D(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
   }

   /**
    * Tests if this the given line segment 3D intersects this bounding box.
    *
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd second endpoint of the line segment. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    */
   public boolean doesIntersectWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      return intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, null, null) > 0;
   }

   /**
    * Tests if this the given ray 3D intersects this bounding box.
    *
    * @param rayOrigin the origin of the ray. Not modified.
    * @param rayDirection the ray direction. Not modified.
    * @return {@code true} if the ray and this bounding box intersect, {@code false} otherwise.
    */
   public boolean doesIntersectWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection)
   {
      return intersectionWithRay3D(rayOrigin, rayDirection, null, null) > 0;
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param line3D the query. Not modified.
    * @param firstIntersectionToPack the coordinate of the first intersection. Can be {@code null}.
    *           Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *           Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    */
   public int intersectionWithLine3D(Line3DReadOnly line3D, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWithLine3D(line3D.getPoint(), line3D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param pointOnLine a point located on the infinitely long line. Not modified.
    * @param lineDirection the line direction. Not modified.
    * @param firstIntersectionToPack the coordinate of the first intersection. Can be {@code null}.
    *           Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *           Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    */
   public int intersectionWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                     Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine3DAndBoundingBox3D(minPoint, maxPoint, pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains
    * unmodified.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegment3D the query. Not modified.
    * @param firstIntersectionToPack the coordinate of the first intersection. Can be {@code null}.
    *           Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *           Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    */
   public int intersectionWithLineSegment3D(LineSegment3DReadOnly lineSegment3D, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment3D(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), firstIntersectionToPack,
                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack the coordinate of the first intersection. Can be {@code null}.
    *           Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *           Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    */
   public int intersectionWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd, Point3DBasics firstIntersectionToPack,
                                            Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLineSegment3DAndBoundingBox3D(minPoint, maxPoint, lineSegmentStart, lineSegmentEnd, firstIntersectionToPack,
                                                              secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a ray and this bounding box.
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and this bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and this bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param rayOrigin the coordinate of the ray origin. Not modified.
    * @param rayDirection the direction of the ray. Not modified.
    * @param firstIntersectionToPack the coordinate of the first intersection. Can be {@code null}.
    *           Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *           Modified.
    * @return the number of intersections between the ray and this bounding box. It is either equal to
    *         0, 1, or 2.
    */
   public int intersectionWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection, Point3DBasics firstIntersectionToPack,
                                    Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenRay3DAndBoundingBox3D(minPoint, maxPoint, rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param point the point to be included in this bounding box. Not modified.
    */
   public void updateToIncludePoint(Point3DReadOnly point)
   {
      this.updateToIncludePoint(point.getX(), point.getY(), point.getZ());
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param x x-coordinate of the point to be included in this bounding box. Not modified.
    * @param y y-coordinate of the point to be included in this bounding box. Not modified.
    * @param z z-coordinate of the point to be included in this bounding box. Not modified.
    */
   public void updateToIncludePoint(double x, double y, double z)
   {
      if (Double.isNaN(minPoint.getX()) || x < minPoint.getX())
      {
         minPoint.setX(x);
      }

      if (Double.isNaN(minPoint.getY()) || y < minPoint.getY())
      {
         minPoint.setY(y);
      }

      if (Double.isNaN(minPoint.getZ()) || z < minPoint.getZ())
      {
         minPoint.setZ(z);
      }

      if (Double.isNaN(maxPoint.getX()) || x > maxPoint.getX())
      {
         maxPoint.setX(x);
      }

      if (Double.isNaN(maxPoint.getY()) || y > maxPoint.getY())
      {
         maxPoint.setY(y);
      }

      if (Double.isNaN(maxPoint.getZ()) || z > maxPoint.getZ())
      {
         maxPoint.setZ(z);
      }
   }

   /**
    * Gets the minimum coordinate of this bounding box and stores it in the given {@code minToPack}.
    *
    * @param minToPack point 3D in which the minimum coordinate of this bounding box is stored.
    *           Modified.
    */
   public void getMinPoint(Point3DBasics minToPack)
   {
      minToPack.set(minPoint);
   }

   /**
    * Gets the maximum coordinate of this bounding box and stores it in the given {@code maxToPack}.
    *
    * @param maxToPack point 3D in which the maximum coordinate of this bounding box is stored.
    *           Modified.
    */
   public void getMaxPoint(Point3DBasics maxToPack)
   {
      maxToPack.set(maxPoint);
   }

   /**
    * Gets the minimum coordinate of this bounding box and stores it in the given array
    * {@code maxToPack}.
    *
    * @param minToPack array in which the minimum coordinate of this bounding box is stored. Modified.
    */
   public void getMinPoint(double[] minToPack)
   {
      minPoint.get(minToPack);
   }

   /**
    * Gets the maximum coordinate of this bounding box and stores it in the given array
    * {@code maxToPack}.
    *
    * @param maxToPack array in which the maximum coordinate of this bounding box is stored. Modified.
    */
   public void getMaxPoint(double[] maxToPack)
   {
      maxPoint.get(maxToPack);
   }

   /**
    * Gets the read-only reference to the minimum coordinate of this bounding box.
    *
    * @return the read-only reference to the minimum coordinate.
    */
   public Point3DReadOnly getMinPoint()
   {
      return minPoint;
   }

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    *
    * @return the read-only reference to the maximum coordinate.
    */
   public Point3DReadOnly getMaxPoint()
   {
      return maxPoint;
   }

   /**
    * Gets the minimum x-coordinate of this bounding box.
    *
    * @return the minimum x-coordinate.
    */
   public double getMinX()
   {
      return minPoint.getX();
   }

   /**
    * Gets the minimum y-coordinate of this bounding box.
    *
    * @return the minimum y-coordinate.
    */
   public double getMinY()
   {
      return minPoint.getY();
   }

   /**
    * Gets the minimum z-coordinate of this bounding box.
    *
    * @return the minimum z-coordinate.
    */
   public double getMinZ()
   {
      return minPoint.getZ();
   }

   /**
    * Gets the maximum x-coordinate of this bounding box.
    *
    * @return the maximum x-coordinate.
    */
   public double getMaxX()
   {
      return maxPoint.getX();
   }

   /**
    * Gets the maximum y-coordinate of this bounding box.
    *
    * @return the maximum y-coordinate.
    */
   public double getMaxY()
   {
      return maxPoint.getY();
   }

   /**
    * Gets the maximum z-coordinate of this bounding box.
    *
    * @return the maximum z-coordinate.
    */
   public double getMaxZ()
   {
      return maxPoint.getZ();
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
      return minPoint.epsilonEquals(other.minPoint, epsilon) && maxPoint.epsilonEquals(other.maxPoint, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(BoundingBox3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((BoundingBox3D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis, if this bounding box 3D is exactly equal to {@code other}.
    *
    * @param other the other bounding box 3D to compare against this. Not modified.
    * @return {@code true} if the two bounding boxes are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(BoundingBox3D other)
   {
      if (other == null)
         return false;
      else
         return minPoint.equals(other.minPoint) && maxPoint.equals(other.maxPoint);
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
      return minPoint.geometricallyEquals(other.minPoint, epsilon) && maxPoint.geometricallyEquals(other.maxPoint, epsilon);
   }
}
