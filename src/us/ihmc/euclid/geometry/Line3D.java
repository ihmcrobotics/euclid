package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely long 3D line defined by a 3D point and a 3D unitary vector.
 */
public class Line3D implements GeometryObject<Line3D>
{
   private final static double minAllowableVectorPart = Math.sqrt(Double.MIN_NORMAL);

   /** Coordinates of a point located on this line. */
   private final Point3D point = new Point3D();
   /** Normalized direction of this line. */
   private final Vector3D direction = new Vector3D();

   private boolean hasPointBeenSet = false;
   private boolean hasDirectionBeenSet = false;

   /**
    * Default constructor that initializes both {@link #point} and {@link #direction} to zero. This
    * point and vector have to be set to valid values to make this line usable.
    */
   public Line3D()
   {
      hasPointBeenSet = false;
      hasDirectionBeenSet = false;
   }

   /**
    * Creates a new line 3D and initializes it to {@code other}.
    * 
    * @param other the other line used to initialize this line. Not modified.
    */
   public Line3D(Line3D other)
   {
      set(other);
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLine point on this line. Not modified.
    * @param lineDirection direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
   }

   /**
    * Initializes this line to be passing through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   public Line3D(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLineX the x-coordinate of a point on this line.
    * @param pointOnLineY the y-coordinate of a point on this line.
    * @param pointOnLineZ the z-coordinate of a point on this line.
    * @param lineDirectionX the x-component of the direction of this line.
    * @param lineDirectionY the y-component of the direction of this line.
    * @param lineDirectionZ the z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line3D(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      set(pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Changes the point through which this line has to go.
    * 
    * @param pointX the new x-coordinate of the point on this line.
    * @param pointY the new y-coordinate of the point on this line.
    * @param pointZ the new z-coordinate of the point on this line.
    */
   public void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ)
   {
      this.point.set(pointOnLineX, pointOnLineY, pointOnLineZ);
      hasPointBeenSet = true;
   }

   /**
    * Changes the point through which this line has to go.
    * 
    * @param pointOnLine new point on this line. Not modified.
    */
   public void setPoint(Point3DReadOnly pointOnLine)
   {
      setPoint(pointOnLine.getX(), pointOnLine.getY(), pointOnLine.getZ());
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    * 
    * @param directionX the new x-component of the direction of this line.
    * @param directionY the new y-component of the direction of this line.
    * @param directionZ the new z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      direction.set(lineDirectionX, lineDirectionY, lineDirectionZ);
      checkReasonableVector(direction);
      direction.normalize();
      hasDirectionBeenSet = true;
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    * 
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void setDirection(Vector3DReadOnly lineDirection)
   {
      setDirection(lineDirection.getX(), lineDirection.getY(), lineDirection.getZ());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * 
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void set(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * 
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param pointOnLineZ the new z-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @param lineDirectionZ the new z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void set(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      setPoint(pointOnLineX, pointOnLineY, pointOnLineZ);
      setDirection(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   public void set(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      checkDistinctPoints(firstPointOnLine, secondPointOnLine);
      setPoint(firstPointOnLine);
      setDirection(secondPointOnLine.getX() - firstPointOnLine.getX(), secondPointOnLine.getY() - firstPointOnLine.getY(),
                   secondPointOnLine.getZ() - firstPointOnLine.getZ());
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * 
    * @param twoPointsOnLine a two-element array containing in order the first point and second
    *           point this line is to go through. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public void set(Point3DReadOnly[] twoPointsOnLine)
   {
      if (twoPointsOnLine.length != 2)
         throw new IllegalArgumentException("Length of input array is not correct. Length = " + twoPointsOnLine.length + ", expected an array of two elements");
      set(twoPointsOnLine[0], twoPointsOnLine[1]);
   }

   /**
    * Sets this line to be the same as the given line.
    * 
    * @param other the other line to copy. Not modified.
    */
   @Override
   public void set(Line3D other)
   {
      set(other.getPoint(), other.getDirection());
   }

   /**
    * Sets the point and vector of this line to zero. After calling this method, this line becomes
    * invalid. A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   public void setToZero()
   {
      point.setToZero();
      direction.setToZero();
      hasPointBeenSet = false;
      hasDirectionBeenSet = false;
   }

   /**
    * Sets the point and vector of this line to {@link Double#NaN}. After calling this method, this
    * line becomes invalid. A new valid point and valid vector will have to be set so this line is
    * again usable.
    */
   @Override
   public void setToNaN()
   {
      point.setToNaN();
      direction.setToNaN();
   }

   /**
    * Tests if this line contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #point} and/or {@link #direction} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return point.containsNaN() || direction.containsNaN();
   }

   /**
    * Computes the minimum distance the given 3D point and this line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code direction.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns the distance
    * between {@code point} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 3D point and this 3D line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double distance(Point3DReadOnly point)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, this.point, this.direction);
   }

   /**
    * This methods computes the minimum distance between this line and {@code otherLine}.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLine the other line to compute the distance from. Not modified.
    * @return the minimum distance between the two lines.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double distance(Line3D otherLine)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceBetweenTwoLine3Ds(point, direction, otherLine.point, otherLine.direction);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double closestPointsWith(Line3D otherLine, Point3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(point, direction, otherLine.point, otherLine.direction, closestPointOnThisLineToPack,
                                                                  closestPointOnOtherLineToPack);
   }

   /**
    * Gets the read-only reference to the point through which this line is going.
    * 
    * @return the reference to the point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point3DReadOnly getPoint()
   {
      checkHasBeenInitialized();
      return point;
   }

   /**
    * Gets the read-only reference to the direction of this line.
    * 
    * @return the reference to the direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Vector3DReadOnly getDirection()
   {
      checkHasBeenInitialized();
      return direction;
   }

   /**
    * Gets the point defining this line by storing its coordinates in the given argument
    * {@code pointToPack}.
    * 
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getPoint(Point3DBasics pointToPack)
   {
      checkHasBeenInitialized();
      pointToPack.set(point);
   }

   /**
    * Gets the direction defining this line by storing its components in the given argument
    * {@code directionToPack}.
    * 
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getDirection(Vector3DBasics directionToPack)
   {
      checkHasBeenInitialized();
      directionToPack.set(direction);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    * 
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getPointAndDirection(Point3DBasics pointToPack, Vector3DBasics directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    * 
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getTwoPointsOnLine(Point3DBasics firstPointOnLineToPack, Point3DBasics secondPointOnLineToPack)
   {
      checkHasBeenInitialized();
      firstPointOnLineToPack.set(point);
      secondPointOnLineToPack.add(point, direction);
   }

   /**
    * Gets the x-coordinate of a point this line goes through.
    * 
    * @return the x-coordinate of this line's point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getPointX()
   {
      checkHasBeenInitialized();
      return point.getX();
   }

   /**
    * Gets the y-coordinate of a point this line goes through.
    * 
    * @return the y-coordinate of this line's point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getPointY()
   {
      checkHasBeenInitialized();
      return point.getY();
   }

   /**
    * Gets the z-coordinate of a point this line goes through.
    * 
    * @return the z-coordinate of this line's point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getPointZ()
   {
      checkHasBeenInitialized();
      return point.getZ();
   }

   /**
    * Gets the x-component of this line's direction.
    * 
    * @return the x-component of this line's direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getDirectionX()
   {
      checkHasBeenInitialized();
      return direction.getX();
   }

   /**
    * Gets the y-component of this line's direction.
    * 
    * @return the y-component of this line's direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getDirectionY()
   {
      checkHasBeenInitialized();
      return direction.getY();
   }

   /**
    * Gets the z-component of this line's direction.
    * 
    * @return the z-component of this line's direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getDirectionZ()
   {
      checkHasBeenInitialized();
      return direction.getZ();
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    * 
    * @param t the parameter used to calculate the point coordinates.
    * @param pointToPack the point in which the coordinates of 'p' are stored. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void pointOnLineGivenParameter(double t, Point3DBasics pointToPack)
   {
      checkHasBeenInitialized();
      pointToPack.scaleAdd(t, direction, point);
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param t the parameter used to calculate the point coordinates.
    * @return the coordinates of the point 'p'.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point3D pointOnLineGivenParameter(double t)
   {
      Point3D pointToReturn = new Point3D();
      pointOnLineGivenParameter(t, pointToReturn);
      return pointToReturn;
   }

   /**
    * Calculates the parameter 't' corresponding to the coordinates of the given {@code pointOnLine}
    * 'p' by solving the line equation:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    * 
    * @param pointOnLine the coordinates of the 'p' from which the parameter 't' is to be
    *           calculated. The point has to be on the line. Not modified.
    * @param epsilon the maximum distance allowed between the given point and this line. If the
    *           given point is at a distance less than {@code epsilon} from this line, it is
    *           considered as being located on this line.
    * @return the value of the parameter 't' corresponding to the given point.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws RuntimeException if the given point is located at a distance greater than
    *            {@code epsilon} from this line.
    */
   public double parameterGivenPointOnLine(Point3DReadOnly pointOnLine, double epsilon)
   {
      if (!isPointOnLine(pointOnLine, epsilon))
      {
         throw new RuntimeException("The given point is not on this line, distance from line: " + distance(pointOnLine));
      }
      else
      {
         double x0 = this.point.getX();
         double y0 = this.point.getY();
         double z0 = this.point.getZ();
         double x1 = x0 + direction.getX();
         double y1 = y0 + direction.getY();
         double z1 = z0 + direction.getZ();
         return EuclidGeometryTools.percentageAlongLineSegment3D(pointOnLine.getX(), pointOnLine.getY(), pointOnLine.getZ(), x0, y0, z0, x1, y1, z1);
      }
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less
    * than {@code epsilon} from it.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnLine(Point3DReadOnly point, double epsilon)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, this.point, direction) < epsilon;
   }

   /**
    * Flips this line's direction.
    * 
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void negateDirection()
   {
      checkHasBeenInitialized();
      direction.negate();
   }

   /**
    * Copies this line and then flips the direction of the copy before returning it.
    * 
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Line3D negateDirectionCopy()
   {
      checkHasBeenInitialized();
      Line3D ret = new Line3D(this);
      ret.negateDirection();

      return ret;
   }

   /**
    * Translates this line by the given (x, y, z).
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    * 
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @param z the distance to translate this line along the z-axis.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void translate(double x, double y, double z)
   {
      checkHasBeenInitialized();
      point.add(x, y, z);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, point, direction, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point3D orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, point, direction);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyTransform(transform);
      direction.applyTransform(transform);
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two
    * lines are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public boolean epsilonEquals(Line3D other, double epsilon)
   {
      checkHasBeenInitialized();
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Line3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Line3D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis, if this line 3D is exactly equal to {@code other}.
    *
    * @param other the other line 3D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Line3D other)
   {
      if (other == null)
         return false;
      else
         return point.equals(other.point) && direction.equals(other.direction);
   }

   private void checkReasonableVector(Vector3DReadOnly localVector)
   {
      if (Math.abs(localVector.getX()) < minAllowableVectorPart && Math.abs(localVector.getY()) < minAllowableVectorPart
            && Math.abs(localVector.getZ()) < minAllowableVectorPart)
      {
         throw new RuntimeException("Line length must be greater than zero");
      }
   }

   private void checkDistinctPoints(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points");
      }
   }

   private void checkHasBeenInitialized()
   {
      if (!hasPointBeenSet)
         throw new RuntimeException("The point of this line has not been initialized.");
      if (!hasDirectionBeenSet)
         throw new RuntimeException("The direction of this line has not been initialized.");
   }

   /**
    * Provides a {@code String} representation of this line 3D as follows:<br>
    * Line 3D: point = (x, y, z), direction = (x, y, z)
    *
    * @return the {@code String} representing this line 3D.
    */
   @Override
   public String toString()
   {
      return "Line 3D: point = " + point + ", direction = " + direction;
   }
}
