package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.Location;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides a large variety of basics geometry operations.
 *
 * @author Sylvain Bertrand
 */
public class EuclidGeometryTools
{
   /** Tolerance used to identify edge cases. */
   public static final double ONE_MILLIONTH = 1.0e-6;
   /** Tolerance used to identify edge cases. */
   public static final double ONE_TEN_MILLIONTH = 1.0e-7;
   /** Tolerance used to identify edge cases. */
   public static final double ONE_TRILLIONTH = 1.0e-12;
   /** Tolerance used to identify edge cases. */
   public static final double IS_POINT_ON_LINE_EPS = 1.0e-8;
   /** Constant used to save some computation. */
   public static final double HALF_PI = 0.5 * Math.PI;

   private EuclidGeometryTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Computes the angle in radians from the first 2D vector to the second 2D vector. The computed
    * angle is in the range [-<i>pi</i>; <i>pi</i>].
    * <p>
    * Redirection to {@link TupleTools#angle(double, double, double, double)}.
    * </p>
    *
    * @param firstVectorX  x-component of the first vector. Not modified.
    * @param firstVectorY  y-component of the first vector. Not modified.
    * @param secondVectorX x-component of the second vector. Not modified.
    * @param secondVectorY y-component of the second vector. Not modified.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double angleFromFirstToSecondVector2D(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY)
   {
      return TupleTools.angle(firstVectorX, firstVectorY, secondVectorX, secondVectorY);
   }

   /**
    * Computes the angle in radians from {@code xForward = (1.0, 0.0)} to the given 2D vector. The
    * computed angle is in the range [-<i>pi</i>; <i>pi</i>].
    *
    * @param vector the vector to compute the angle of. Not modified.
    * @return the angle in radians from xForward to the vector.
    */
   public static double angleFromXForwardToVector2D(Vector2DReadOnly vector)
   {
      return Axis2D.X.angle(vector);
   }

   /**
    * Computes the angle in radians from {@code xForward = (1.0, 0.0)} to the given 2D vector. The
    * computed angle is in the range [-<i>pi</i>; <i>pi</i>].
    *
    * @param vectorX x-component of the vector to compute the angle of.
    * @param vectorY y-component of the vector to compute the angle of.
    * @return the angle in radians from xForward to the vector.
    */
   public static double angleFromXForwardToVector2D(double vectorX, double vectorY)
   {
      return angleFromFirstToSecondVector2D(1.0, 0.0, vectorX, vectorY);
   }

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector. The computed
    * angle is in the range [0; <i>pi</i>].
    * <p>
    * Redirection to {@link TupleTools#angle(double, double, double, double, double, double)}.
    * </p>
    *
    * @param firstVectorX  x-component of first the vector.
    * @param firstVectorY  y-component of first the vector.
    * @param firstVectorZ  z-component of first the vector.
    * @param secondVectorX x-component of second the vector.
    * @param secondVectorY y-component of second the vector.
    * @param secondVectorZ z-component of second the vector.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double angleFromFirstToSecondVector3D(double firstVectorX,
                                                       double firstVectorY,
                                                       double firstVectorZ,
                                                       double secondVectorX,
                                                       double secondVectorY,
                                                       double secondVectorZ)
   {
      return TupleTools.angle(firstVectorX, firstVectorY, firstVectorZ, secondVectorX, secondVectorY, secondVectorZ);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1x   x-coordinate of a point located on the first line.
    * @param pointOnLine1y   y-coordinate of a point located on the first line.
    * @param lineDirection1x x-component of the first line direction.
    * @param lineDirection1y y-component of the first line direction.
    * @param pointOnLine2x   x-coordinate of a point located on the second line.
    * @param pointOnLine2y   y-coordinate of a point located on the second line.
    * @param lineDirection2x x-component of the second line direction.
    * @param lineDirection2y y-component of the second line direction.
    * @param angleEpsilon    tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to
    *                        the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areLine2DsCollinear(double pointOnLine1x,
                                             double pointOnLine1y,
                                             double lineDirection1x,
                                             double lineDirection1y,
                                             double pointOnLine2x,
                                             double pointOnLine2y,
                                             double lineDirection2x,
                                             double lineDirection2y,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      return areLine2DsCollinear(pointOnLine1x,
                                 pointOnLine1y,
                                 lineDirection1x,
                                 lineDirection1y,
                                 false,
                                 pointOnLine2x,
                                 pointOnLine2y,
                                 lineDirection2x,
                                 lineDirection2y,
                                 false,
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   private static boolean areLine2DsCollinear(double pointOnLine1x,
                                              double pointOnLine1y,
                                              double lineDirection1x,
                                              double lineDirection1y,
                                              boolean isDirection1Unitary,
                                              double pointOnLine2x,
                                              double pointOnLine2y,
                                              double lineDirection2x,
                                              double lineDirection2y,
                                              boolean isDirection2Unitary,
                                              double angleEpsilon,
                                              double distanceEpsilon)
   {
      if (!areVector2DsParallel(lineDirection1x, lineDirection1y, isDirection1Unitary, lineDirection2x, lineDirection2y, isDirection2Unitary, angleEpsilon))
         return false;

      double distance = distanceFromPoint2DToLine2D(pointOnLine2x,
                                                    pointOnLine2y,
                                                    pointOnLine1x,
                                                    pointOnLine1y,
                                                    lineDirection1x,
                                                    lineDirection1y,
                                                    isDirection1Unitary);
      return distance < distanceEpsilon;
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon       tolerance on the angle in radians.
    * @param distanceEpsilon    tolerance on the distance to determine if {@code firstPointOnLine2}
    *                           belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areLine2DsCollinear(Point2DReadOnly firstPointOnLine1,
                                             Point2DReadOnly secondPointOnLine1,
                                             Point2DReadOnly firstPointOnLine2,
                                             Point2DReadOnly secondPointOnLine2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      return areLine2DsCollinear(pointOnLine1x,
                                 pointOnLine1y,
                                 lineDirection1x,
                                 lineDirection1y,
                                 pointOnLine2x,
                                 pointOnLine2y,
                                 lineDirection2x,
                                 lineDirection2y,
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1       point located on the first line. Not modified.
    * @param lineDirection1     the first line direction. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon       tolerance on the angle in radians.
    * @param distanceEpsilon    tolerance on the distance to determine if {@code firstPointOnLine2}
    *                           belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areLine2DsCollinear(Point2DReadOnly pointOnLine1,
                                             Vector2DReadOnly lineDirection1,
                                             Point2DReadOnly firstPointOnLine2,
                                             Point2DReadOnly secondPointOnLine2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      return areLine2DsCollinear(pointOnLine1.getX(),
                                 pointOnLine1.getY(),
                                 lineDirection1.getX(),
                                 lineDirection1.getY(),
                                 lineDirection1 instanceof UnitVector2DReadOnly,
                                 pointOnLine2x,
                                 pointOnLine2y,
                                 lineDirection2x,
                                 lineDirection2y,
                                 false,
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1    point located on the first line. Not modified.
    * @param lineDirection1  the first line direction. Not modified.
    * @param pointOnLine2    point located on the second line. Not modified.
    * @param lineDirection2  the second line direction. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to
    *                        the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areLine2DsCollinear(Point2DReadOnly pointOnLine1,
                                             Vector2DReadOnly lineDirection1,
                                             Point2DReadOnly pointOnLine2,
                                             Vector2DReadOnly lineDirection2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      return areLine2DsCollinear(pointOnLine1.getX(),
                                 pointOnLine1.getY(),
                                 lineDirection1.getX(),
                                 lineDirection1.getY(),
                                 pointOnLine2.getX(),
                                 pointOnLine2.getY(),
                                 lineDirection2.getX(),
                                 lineDirection2.getY(),
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1x   x-coordinate of a point located on the first line.
    * @param pointOnLine1y   y-coordinate of a point located on the first line.
    * @param pointOnLine1z   z-coordinate of a point located on the first line.
    * @param lineDirection1x x-component of the first line direction.
    * @param lineDirection1y y-component of the first line direction.
    * @param lineDirection1z z-component of the first line direction.
    * @param pointOnLine2x   x-coordinate of a point located on the second line.
    * @param pointOnLine2y   y-coordinate of a point located on the second line.
    * @param pointOnLine2z   z-coordinate of a point located on the second line.
    * @param lineDirection2x x-component of the second line direction.
    * @param lineDirection2y y-component of the second line direction.
    * @param lineDirection2z z-component of the second line direction.
    * @param angleEpsilon    tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to
    *                        the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areLine3DsCollinear(double pointOnLine1x,
                                             double pointOnLine1y,
                                             double pointOnLine1z,
                                             double lineDirection1x,
                                             double lineDirection1y,
                                             double lineDirection1z,
                                             double pointOnLine2x,
                                             double pointOnLine2y,
                                             double pointOnLine2z,
                                             double lineDirection2x,
                                             double lineDirection2y,
                                             double lineDirection2z,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      return areLine3DsCollinear(pointOnLine1x,
                                 pointOnLine1y,
                                 pointOnLine1z,
                                 lineDirection1x,
                                 lineDirection1y,
                                 lineDirection1z,
                                 false,
                                 pointOnLine2x,
                                 pointOnLine2y,
                                 pointOnLine2z,
                                 lineDirection2x,
                                 lineDirection2y,
                                 lineDirection2z,
                                 false,
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   private static boolean areLine3DsCollinear(double pointOnLine1x,
                                              double pointOnLine1y,
                                              double pointOnLine1z,
                                              double lineDirection1x,
                                              double lineDirection1y,
                                              double lineDirection1z,
                                              boolean isDirection1Unitary,
                                              double pointOnLine2x,
                                              double pointOnLine2y,
                                              double pointOnLine2z,
                                              double lineDirection2x,
                                              double lineDirection2y,
                                              double lineDirection2z,
                                              boolean isDirection2Unitary,
                                              double angleEpsilon,
                                              double distanceEpsilon)
   {
      if (!areVector3DsParallel(lineDirection1x,
                                lineDirection1y,
                                lineDirection1z,
                                isDirection1Unitary,
                                lineDirection2x,
                                lineDirection2y,
                                lineDirection2z,
                                isDirection2Unitary,
                                angleEpsilon))
         return false;

      double distance = distanceFromPoint3DToLine3D(pointOnLine2x,
                                                    pointOnLine2y,
                                                    pointOnLine2z,
                                                    pointOnLine1x,
                                                    pointOnLine1y,
                                                    pointOnLine1z,
                                                    lineDirection1x,
                                                    lineDirection1y,
                                                    lineDirection1z,
                                                    isDirection1Unitary);
      return distance < distanceEpsilon;
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon       tolerance on the angle in radians.
    * @param distanceEpsilon    tolerance on the distance to determine if {@code firstPointOnLine2}
    *                           belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areLine3DsCollinear(Point3DReadOnly firstPointOnLine1,
                                             Point3DReadOnly secondPointOnLine1,
                                             Point3DReadOnly firstPointOnLine2,
                                             Point3DReadOnly secondPointOnLine2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double pointOnLine1z = firstPointOnLine1.getZ();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double lineDirection1z = secondPointOnLine1.getZ() - firstPointOnLine1.getZ();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double pointOnLine2z = firstPointOnLine2.getZ();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      double lineDirection2z = secondPointOnLine2.getZ() - firstPointOnLine2.getZ();
      return areLine3DsCollinear(pointOnLine1x,
                                 pointOnLine1y,
                                 pointOnLine1z,
                                 lineDirection1x,
                                 lineDirection1y,
                                 lineDirection1z,
                                 pointOnLine2x,
                                 pointOnLine2y,
                                 pointOnLine2z,
                                 lineDirection2x,
                                 lineDirection2y,
                                 lineDirection2z,
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below {@link #ONE_TEN_MILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1    point located on the first line. Not modified.
    * @param lineDirection1  the first line direction. Not modified.
    * @param pointOnLine2    point located on the second line. Not modified.
    * @param lineDirection2  the second line direction. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to
    *                        the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLine3DsCollinear(Point3DReadOnly pointOnLine1,
                                             Vector3DReadOnly lineDirection1,
                                             Point3DReadOnly pointOnLine2,
                                             Vector3DReadOnly lineDirection2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      return areLine3DsCollinear(pointOnLine1.getX(),
                                 pointOnLine1.getY(),
                                 pointOnLine1.getZ(),
                                 lineDirection1.getX(),
                                 lineDirection1.getY(),
                                 lineDirection1.getZ(),
                                 lineDirection1 instanceof UnitVector3DReadOnly,
                                 pointOnLine2.getX(),
                                 pointOnLine2.getY(),
                                 pointOnLine2.getZ(),
                                 lineDirection2.getX(),
                                 lineDirection2.getY(),
                                 lineDirection2.getZ(),
                                 lineDirection2 instanceof UnitVector3DReadOnly,
                                 angleEpsilon,
                                 distanceEpsilon);
   }

   /**
    * Tests if the two given planes are coincident:
    * <ul>
    * <li>{@code planeNormal1} and {@code planeNormal2} are parallel given the tolerance
    * {@code angleEpsilon}.
    * <li>the distance of {@code pointOnPlane2} from the first plane is less than
    * {@code distanceEpsilon}.
    * </ul>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@link #ONE_TEN_MILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1   a point on the first plane. Not modified.
    * @param planeNormal1    the normal of the first plane. Not modified.
    * @param pointOnPlane2   a point on the second plane. Not modified.
    * @param planeNormal2    the normal of the second plane. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians to determine if the plane normals are
    *                        parallel.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnPlane2} belongs to
    *                        the first plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean arePlane3DsCoincident(Point3DReadOnly pointOnPlane1,
                                               Vector3DReadOnly planeNormal1,
                                               Point3DReadOnly pointOnPlane2,
                                               Vector3DReadOnly planeNormal2,
                                               double angleEpsilon,
                                               double distanceEpsilon)
   {
      if (!areVector3DsParallel(planeNormal1, planeNormal2, angleEpsilon))
         return false;
      else
         return distanceFromPoint3DToPlane3D(pointOnPlane2, pointOnPlane1, planeNormal1) < distanceEpsilon;
   }

   /**
    * Tests if the two given vectors are parallel given a tolerance on the angle between the two vector
    * axes in the range ]0; <i>pi</i>/2[. This method returns {@code true} if the two vectors are
    * parallel, whether they are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@link #ONE_TEN_MILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstVectorX  x-component of the first vector. Not modified.
    * @param firstVectorY  y-component of the first vector. Not modified.
    * @param secondVectorX x-component of the second vector. Not modified.
    * @param secondVectorY y-component of the second vector. Not modified.
    * @param angleEpsilon  tolerance on the angle in radians.
    * @return {@code true} if the two vectors are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areVector2DsParallel(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY, double angleEpsilon)
   {
      return areVector2DsParallel(firstVectorX, firstVectorY, false, secondVectorX, secondVectorY, false, angleEpsilon);
   }

   private static boolean areVector2DsParallel(double firstVectorX,
                                               double firstVectorY,
                                               boolean isFirstVectorUnitary,
                                               double secondVectorX,
                                               double secondVectorY,
                                               boolean isSecondVectorUnitary,
                                               double angleEpsilon)
   {
      if (angleEpsilon < 0.0 || angleEpsilon > HALF_PI)
         throw new IllegalArgumentException("The angle epsilon has to be inside the interval: [0.0 ; Math.PI / 2.0]");

      double firstVectorLength = isFirstVectorUnitary ? 1.0 : EuclidCoreTools.norm(firstVectorX, firstVectorY);
      if (firstVectorLength < ONE_TEN_MILLIONTH)
         return false;
      double secondVectorLength = isSecondVectorUnitary ? 1.0 : EuclidCoreTools.norm(secondVectorX, secondVectorY);
      if (secondVectorLength < ONE_TEN_MILLIONTH)
         return false;
      double dot = firstVectorX * secondVectorX + firstVectorY * secondVectorY;
      return Math.abs(dot / (firstVectorLength * secondVectorLength)) > EuclidCoreTools.cos(angleEpsilon);
   }

   /**
    * Tests if the two given vectors are parallel given a tolerance on the angle between the two vector
    * axes in the range ]0; <i>pi</i>/2[. This method returns {@code true} if the two vectors are
    * parallel, whether they are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@link #ONE_TEN_MILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areVector2DsParallel(Vector2DReadOnly firstVector, Vector2DReadOnly secondVector, double angleEpsilon)
   {
      return areVector2DsParallel(firstVector.getX(),
                                  firstVector.getY(),
                                  firstVector instanceof UnitVector2DReadOnly,
                                  secondVector.getX(),
                                  secondVector.getY(),
                                  secondVector instanceof UnitVector2DReadOnly,
                                  angleEpsilon);
   }

   /**
    * Tests if the two given vectors are parallel given a tolerance on the angle between the two vector
    * axes in the range ]0; <i>pi</i>/2[. This method returns {@code true} if the two vectors are
    * parallel, whether they are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@link #ONE_TEN_MILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstVectorX  x-component of the first vector. Not modified.
    * @param firstVectorY  y-component of the first vector. Not modified.
    * @param firstVectorZ  z-component of the first vector. Not modified.
    * @param secondVectorX x-component of the second vector. Not modified.
    * @param secondVectorY y-component of the second vector. Not modified.
    * @param secondVectorZ z-component of the second vector. Not modified.
    * @param angleEpsilon  tolerance on the angle in radians.
    * @return {@code true} if the two vectors are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areVector3DsParallel(double firstVectorX,
                                              double firstVectorY,
                                              double firstVectorZ,
                                              double secondVectorX,
                                              double secondVectorY,
                                              double secondVectorZ,
                                              double angleEpsilon)
   {
      return areVector3DsParallel(firstVectorX, firstVectorY, firstVectorZ, false, secondVectorX, secondVectorY, secondVectorZ, false, angleEpsilon);
   }

   private static boolean areVector3DsParallel(double firstVectorX,
                                               double firstVectorY,
                                               double firstVectorZ,
                                               boolean isFirstVectorUnitary,
                                               double secondVectorX,
                                               double secondVectorY,
                                               double secondVectorZ,
                                               boolean isSecondVectorUnitary,
                                               double angleEpsilon)
   {
      if (angleEpsilon < 0.0 || angleEpsilon > HALF_PI)
         throw new IllegalArgumentException("The angle epsilon has to be inside the interval: [0.0 ; Math.PI / 2.0]");

      double firstVectorLength = isFirstVectorUnitary ? 1.0 : EuclidCoreTools.norm(firstVectorX, firstVectorY, firstVectorZ);
      if (firstVectorLength < ONE_TEN_MILLIONTH)
         return false;
      double secondVectorLength = isSecondVectorUnitary ? 1.0 : EuclidCoreTools.norm(secondVectorX, secondVectorY, secondVectorZ);
      if (secondVectorLength < ONE_TEN_MILLIONTH)
         return false;
      double dot = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      return Math.abs(dot / (firstVectorLength * secondVectorLength)) > EuclidCoreTools.cos(angleEpsilon);
   }

   /**
    * Tests if the two given vectors are parallel given a tolerance on the angle between the two vector
    * axes in the range ]0; <i>pi</i>/2[. This method returns {@code true} if the two vectors are
    * parallel, whether they are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@link #ONE_TEN_MILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean areVector3DsParallel(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector, double angleEpsilon)
   {
      return areVector3DsParallel(firstVector.getX(),
                                  firstVector.getY(),
                                  firstVector.getZ(),
                                  firstVector instanceof UnitVector3DReadOnly,
                                  secondVector.getX(),
                                  secondVector.getY(),
                                  secondVector.getZ(),
                                  secondVector instanceof UnitVector3DReadOnly,
                                  angleEpsilon);
   }

   /**
    * Computes the average 2D point from a given collection of 2D points.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param points the collection of 2D points to compute the average from. Not modified.
    * @return the computed average.
    */
   public static Point2D averagePoint2Ds(Collection<? extends Point2DReadOnly> points)
   {
      if (points.isEmpty())
         return null;

      Point2D totalPoint = new Point2D();

      for (Point2DReadOnly point : points)
      {
         totalPoint.add(point);
      }

      totalPoint.scale(1.0 / points.size());

      return totalPoint;
   }

   /**
    * Computes the average 3D point from a given collection of 3D points.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param points the collection of 3D points to compute the average from. Not modified.
    * @return the computed average.
    */
   public static Point3D averagePoint3Ds(Collection<? extends Point3DReadOnly> points)
   {
      if (points.isEmpty())
         return null;

      Point3D totalPoint = new Point3D();

      for (Point3DReadOnly point : points)
      {
         totalPoint.add(point);
      }

      totalPoint.scale(1.0 / points.size());

      return totalPoint;
   }

   /**
    * Returns the average of two 3D points.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param a the first 3D point. Not modified.
    * @param b the second 3D point. Not modified.
    * @return the computed average.
    */
   public static Point3D averagePoint3Ds(Point3DReadOnly a, Point3DReadOnly b)
   {
      Point3D average = new Point3D(a);
      average.add(b);
      average.scale(0.5);

      return average;
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * packs it into an {@link Orientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@link #ONE_TEN_MILLIONTH}: the rotation angle is
    * equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param firstVectorX   x-component of the first vector.
    * @param firstVectorY   y-component of the first vector.
    * @param firstVectorZ   z-component of the first vector.
    * @param secondVectorX  x-component of the second vector that is rotated with respect to the first
    *                       vector.
    * @param secondVectorY  y-component of the second vector that is rotated with respect to the first
    *                       vector.
    * @param secondVectorZ  z-component of the second vector that is rotated with respect to the first
    *                       vector.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    */
   public static void orientation3DFromFirstToSecondVector3D(double firstVectorX,
                                                             double firstVectorY,
                                                             double firstVectorZ,
                                                             double secondVectorX,
                                                             double secondVectorY,
                                                             double secondVectorZ,
                                                             Orientation3DBasics rotationToPack)
   {
      orientation3DFromFirstToSecondVector3D(firstVectorX,
                                             firstVectorY,
                                             firstVectorZ,
                                             false,
                                             secondVectorX,
                                             secondVectorY,
                                             secondVectorZ,
                                             false,
                                             rotationToPack);
   }

   private static void orientation3DFromFirstToSecondVector3D(double firstVectorX,
                                                              double firstVectorY,
                                                              double firstVectorZ,
                                                              boolean isFirstVectorUnitary,
                                                              double secondVectorX,
                                                              double secondVectorY,
                                                              double secondVectorZ,
                                                              boolean isSecondVectorUnitary,
                                                              Orientation3DBasics rotationToPack)
   {
      if (!isFirstVectorUnitary)
      {
         double firstVectorNormSquared = normSquared(firstVectorX, firstVectorY, firstVectorZ);

         if (!EuclidCoreTools.epsilonEquals(firstVectorNormSquared, 1.0, ONE_MILLIONTH))
         {
            double firstVectorInvLength = 1.0 / EuclidCoreTools.squareRoot(firstVectorNormSquared);
            firstVectorX *= firstVectorInvLength;
            firstVectorY *= firstVectorInvLength;
            firstVectorZ *= firstVectorInvLength;
         }
      }

      if (!isSecondVectorUnitary)
      {
         double secondVectorNormSquared = normSquared(secondVectorX, secondVectorY, secondVectorZ);
         if (!EuclidCoreTools.epsilonEquals(secondVectorNormSquared, 1.0, ONE_MILLIONTH))
         {
            double secondVectorInvLength = 1.0 / EuclidCoreTools.squareRoot(secondVectorNormSquared);
            secondVectorX *= secondVectorInvLength;
            secondVectorY *= secondVectorInvLength;
            secondVectorZ *= secondVectorInvLength;
         }
      }

      double axisX = firstVectorY * secondVectorZ - firstVectorZ * secondVectorY;
      double axisY = firstVectorZ * secondVectorX - firstVectorX * secondVectorZ;
      double axisZ = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      double crossNormSquared = normSquared(axisX, axisY, axisZ);

      boolean vectorsAreParallel = crossNormSquared < ONE_TEN_MILLIONTH * ONE_TEN_MILLIONTH;

      if (vectorsAreParallel)
      {
         double dot;
         dot = secondVectorX * firstVectorX;
         dot += secondVectorY * firstVectorY;
         dot += secondVectorZ * firstVectorZ;
         if (dot > 0.0)
            rotationToPack.setToZero();
         else
            rotationToPack.setQuaternion(1.0, 0.0, 0.0, 0.0);
         return;
      }

      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      double qs = 1.0 + dotProduct;
      double norm = 1.0 / EuclidCoreTools.norm(axisX, axisY, axisZ, qs);
      rotationToPack.setQuaternion(axisX * norm, axisY * norm, axisZ * norm, qs * norm);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * packs it into an {@link Orientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param firstVector    the first vector. Not modified.
    * @param secondVector   the second vector that is rotated with respect to the first vector. Not
    *                       modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    */
   public static void orientation3DFromFirstToSecondVector3D(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector, Orientation3DBasics rotationToPack)
   {
      orientation3DFromFirstToSecondVector3D(firstVector.getX(),
                                             firstVector.getY(),
                                             firstVector.getZ(),
                                             firstVector instanceof UnitVector3DReadOnly,
                                             secondVector.getX(),
                                             secondVector.getY(),
                                             secondVector.getZ(),
                                             secondVector instanceof UnitVector3DReadOnly,
                                             rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * returns the result as an {@link AxisAngle}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector that is rotated with respect to the first vector. Not
    *                     modified.
    * @return the minimum rotation from {@code firstVector} to the given {@code secondVector}.
    */
   public static AxisAngle axisAngleFromFirstToSecondVector3D(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector)
   {
      AxisAngle axisAngle = new AxisAngle();
      orientation3DFromFirstToSecondVector3D(firstVector, secondVector, axisAngle);
      return axisAngle;
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given {@code vector}
    * and returns the result as an {@link AxisAngle}.
    * <p>
    * The rotation angle is computed as the angle from the {@code zUp} to the {@code vector}: <br>
    * {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the
    * rotation axis is set to: (1, 0, 0).
    * <li>the vector is parallel pointing opposite direction of {@code zUp}: the rotation angle is
    * equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vector the 3D vector that is rotated with respect to {@code zUp}. Not modified.
    * @return the minimum rotation from {@code zUp} to the given {@code vector}.
    */
   public static AxisAngle axisAngleFromZUpToVector3D(Vector3DReadOnly vector)
   {
      AxisAngle axisAngle = new AxisAngle();
      orientation3DFromZUpToVector3D(vector, axisAngle);
      return axisAngle;
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given {@code vector}
    * and packs it into an {@link Orientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code zUp} to the {@code vector}: <br>
    * {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the
    * rotation axis is set to: (1, 0, 0).
    * <li>the vector is parallel pointing opposite direction of {@code zUp}: the rotation angle is
    * equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param vector         the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}.
    *                       Modified.
    */
   public static void orientation3DFromZUpToVector3D(Vector3DReadOnly vector, Orientation3DBasics rotationToPack)
   {
      orientation3DFromFirstToSecondVector3D(Axis3D.Z, vector, rotationToPack);
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2
    * such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param pointOnLine1              a 3D point on the first line. Not modified.
    * @param lineDirection1            the 3D direction of the first line. Not modified.
    * @param pointOnLine2              a 3D point on the second line. Not modified.
    * @param lineDirection2            the 3D direction of the second line. Not modified.
    * @param closestPointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point.
    *                                  Modified. Can be {@code null}.
    * @param closestPointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point.
    *                                  Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   public static double closestPoint3DsBetweenTwoLine3Ds(Point3DReadOnly pointOnLine1,
                                                         Vector3DReadOnly lineDirection1,
                                                         Point3DReadOnly pointOnLine2,
                                                         Vector3DReadOnly lineDirection2,
                                                         Point3DBasics closestPointOnLine1ToPack,
                                                         Point3DBasics closestPointOnLine2ToPack)
   {
      // Switching to the notation used in http://geomalgorithms.com/a07-_distance.html.
      // The line1 is defined by (P0, u) and the line2 by (Q0, v).
      // Note: the algorithm is independent from the magnitudes of lineDirection1 and lineDirection2
      Point3DReadOnly P0 = pointOnLine1;
      Vector3DReadOnly u = lineDirection1;
      Point3DReadOnly Q0 = pointOnLine2;
      Vector3DReadOnly v = lineDirection2;

      Point3DBasics Psc = closestPointOnLine1ToPack;
      Point3DBasics Qtc = closestPointOnLine2ToPack;

      double w0X = P0.getX() - Q0.getX();
      double w0Y = P0.getY() - Q0.getY();
      double w0Z = P0.getZ() - Q0.getZ();

      double a = u.dot(u);
      double b = u.dot(v);
      double c = v.dot(v);
      double d = u.getX() * w0X + u.getY() * w0Y + u.getZ() * w0Z;
      double e = v.getX() * w0X + v.getY() * w0Y + v.getZ() * w0Z;

      double delta = a * c - b * b;

      double sc, tc;

      // check to see if the lines are parallel
      if (delta <= ONE_TRILLIONTH)
      {
         /*
          * The lines are parallel, there's an infinite number of pairs, but for one chosen point on one of
          * the lines, there's only one closest point to it on the other line. So let's choose arbitrarily a
          * point on the line1 and calculate the point that is closest to it on the line2.
          */
         sc = 0.0;
         tc = d / b;
      }
      else
      {
         sc = (b * e - c * d) / delta;
         tc = (a * e - b * d) / delta;
      }

      double PscX = sc * u.getX() + P0.getX();
      double PscY = sc * u.getY() + P0.getY();
      double PscZ = sc * u.getZ() + P0.getZ();

      double QtcX = tc * v.getX() + Q0.getX();
      double QtcY = tc * v.getY() + Q0.getY();
      double QtcZ = tc * v.getZ() + Q0.getZ();

      if (Psc != null)
         Psc.set(PscX, PscY, PscZ);
      if (Qtc != null)
         Qtc.set(QtcX, QtcY, QtcZ);

      double dx = PscX - QtcX;
      double dy = PscY - QtcY;
      double dz = PscZ - QtcZ;
      double distanceSquared = dx * dx + dy * dy + dz * dz;
      return EuclidCoreTools.squareRoot(distanceSquared);
   }

   /**
    * Given two 2D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 2D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 2D coordinates of the point P are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 2D coordinates of the point Q are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    */
   public static double closestPoint2DsBetweenTwoLineSegment2Ds(Point2DReadOnly lineSegmentStart1,
                                                                Point2DReadOnly lineSegmentEnd1,
                                                                Point2DReadOnly lineSegmentStart2,
                                                                Point2DReadOnly lineSegmentEnd2,
                                                                Point2DBasics closestPointOnLineSegment1ToPack,
                                                                Point2DBasics closestPointOnLineSegment2ToPack)
   {
      return closestPoint2DsBetweenTwoLineSegment2Ds(lineSegmentStart1.getX(),
                                                     lineSegmentStart1.getY(),
                                                     lineSegmentEnd1.getX(),
                                                     lineSegmentEnd1.getY(),
                                                     lineSegmentStart2.getX(),
                                                     lineSegmentStart2.getY(),
                                                     lineSegmentEnd2.getX(),
                                                     lineSegmentEnd2.getY(),
                                                     closestPointOnLineSegment1ToPack,
                                                     closestPointOnLineSegment2ToPack);
   }

   /**
    * Given two 2D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 2D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1X               the x-coordinate of the first endpoint of the first line
    *                                         segment.
    * @param lineSegmentStart1Y               the y-coordinate of the first endpoint of the first line
    *                                         segment.
    * @param lineSegmentEnd1X                 the x-coordinate of the second endpoint of the first line
    *                                         segment.
    * @param lineSegmentEnd1Y                 the y-coordinate of the second endpoint of the first line
    *                                         segment.
    * @param lineSegmentStart2X               the x-coordinate of the first endpoint of the second line
    *                                         segment.
    * @param lineSegmentStart2Y               the y-coordinate of the first endpoint of the second line
    *                                         segment.
    * @param lineSegmentEnd2X                 the x-coordinate of the second endpoint of the second
    *                                         line segment.
    * @param lineSegmentEnd2Y                 the y-coordinate of the second endpoint of the second
    *                                         line segment.
    * @param closestPointOnLineSegment1ToPack the 2D coordinates of the point P are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 2D coordinates of the point Q are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    */
   public static double closestPoint2DsBetweenTwoLineSegment2Ds(double lineSegmentStart1X,
                                                                double lineSegmentStart1Y,
                                                                double lineSegmentEnd1X,
                                                                double lineSegmentEnd1Y,
                                                                double lineSegmentStart2X,
                                                                double lineSegmentStart2Y,
                                                                double lineSegmentEnd2X,
                                                                double lineSegmentEnd2Y,
                                                                Point2DBasics closestPointOnLineSegment1ToPack,
                                                                Point2DBasics closestPointOnLineSegment2ToPack)
   {
      // Switching to the notation used in http://geomalgorithms.com/a07-_distance.html.
      // The line1 is defined by (P0, u) and the line2 by (Q0, v).
      double P0X = lineSegmentStart1X;
      double P0Y = lineSegmentStart1Y;
      double ux = lineSegmentEnd1X - lineSegmentStart1X;
      double uy = lineSegmentEnd1Y - lineSegmentStart1Y;
      double Q0X = lineSegmentStart2X;
      double Q0Y = lineSegmentStart2Y;
      double vx = lineSegmentEnd2X - lineSegmentStart2X;
      double vy = lineSegmentEnd2Y - lineSegmentStart2Y;

      Point2DBasics Psc = closestPointOnLineSegment1ToPack;
      Point2DBasics Qtc = closestPointOnLineSegment2ToPack;

      double w0X = P0X - Q0X;
      double w0Y = P0Y - Q0Y;

      double a = ux * ux + uy * uy;
      double b = ux * vx + uy * vy;
      double c = vx * vx + vy * vy;
      double d = ux * w0X + uy * w0Y;
      double e = vx * w0X + vy * w0Y;

      double ac = a * c;
      double bb = b * b;

      double delta = ac - bb;

      double sc, sNumerator, sDenominator = delta;
      double tc, tNumerator, tDenominator = delta;

      // check to see if the lines are parallel
      if (delta <= ONE_TRILLIONTH * Math.max(ac, bb))
      {
         /*
          * The lines are parallel, there's an infinite number of pairs, but for one chosen point on one of
          * the lines, there's only one closest point to it on the other line. So let's choose arbitrarily a
          * point on the lineSegment1 and calculate the point that is closest to it on the lineSegment2.
          */
         sNumerator = 0.0;
         sDenominator = 1.0;
         tNumerator = e;
         tDenominator = c;
      }
      else
      {
         sNumerator = b * e - c * d;
         tNumerator = a * e - b * d;

         if (sNumerator < 0.0)
         {
            sNumerator = 0.0;
            tNumerator = e;
            tDenominator = c;
         }
         else if (sNumerator > sDenominator)
         {
            sNumerator = sDenominator;
            tNumerator = e + b;
            tDenominator = c;
         }
      }

      if (tNumerator < 0.0)
      {
         tNumerator = 0.0;
         sNumerator = -d;
         if (sNumerator < 0.0)
            sNumerator = 0.0;
         else if (sNumerator > a)
            sNumerator = a;
         sDenominator = a;
      }
      else if (tNumerator > tDenominator)
      {
         tNumerator = tDenominator;
         sNumerator = -d + b;
         if (sNumerator < 0.0)
            sNumerator = 0.0;
         else if (sNumerator > a)
            sNumerator = a;
         sDenominator = a;
      }

      sc = Math.abs(sNumerator) < ONE_TRILLIONTH ? 0.0 : sNumerator / sDenominator;
      tc = Math.abs(tNumerator) < ONE_TRILLIONTH ? 0.0 : tNumerator / tDenominator;

      double PscX = sc * ux + P0X;
      double PscY = sc * uy + P0Y;

      double QtcX = tc * vx + Q0X;
      double QtcY = tc * vy + Q0Y;

      if (Psc != null)
         Psc.set(PscX, PscY);
      if (Qtc != null)
         Qtc.set(QtcX, QtcY);

      double dx = PscX - QtcX;
      double dy = PscY - QtcY;
      return EuclidCoreTools.norm(dx, dy);
   }

   /**
    * Given two 3D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 3D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 3D coordinates of the point P are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 3D coordinates of the point Q are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    */
   public static double closestPoint3DsBetweenTwoLineSegment3Ds(Point3DReadOnly lineSegmentStart1,
                                                                Point3DReadOnly lineSegmentEnd1,
                                                                Point3DReadOnly lineSegmentStart2,
                                                                Point3DReadOnly lineSegmentEnd2,
                                                                Point3DBasics closestPointOnLineSegment1ToPack,
                                                                Point3DBasics closestPointOnLineSegment2ToPack)
   {
      return closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1.getX(),
                                                     lineSegmentStart1.getY(),
                                                     lineSegmentStart1.getZ(),
                                                     lineSegmentEnd1.getX(),
                                                     lineSegmentEnd1.getY(),
                                                     lineSegmentEnd1.getZ(),
                                                     lineSegmentStart2.getX(),
                                                     lineSegmentStart2.getY(),
                                                     lineSegmentStart2.getZ(),
                                                     lineSegmentEnd2.getX(),
                                                     lineSegmentEnd2.getY(),
                                                     lineSegmentEnd2.getZ(),
                                                     closestPointOnLineSegment1ToPack,
                                                     closestPointOnLineSegment2ToPack);
   }

   /**
    * Given two 3D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 3D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1X               the x-coordinate of the first endpoint of the first line
    *                                         segment.
    * @param lineSegmentStart1Y               the y-coordinate of the first endpoint of the first line
    *                                         segment.
    * @param lineSegmentStart1Z               the z-coordinate of the first endpoint of the first line
    *                                         segment.
    * @param lineSegmentEnd1X                 the x-coordinate of the second endpoint of the first line
    *                                         segment.
    * @param lineSegmentEnd1Y                 the y-coordinate of the second endpoint of the first line
    *                                         segment.
    * @param lineSegmentEnd1Z                 the z-coordinate of the second endpoint of the first line
    *                                         segment.
    * @param lineSegmentStart2X               the x-coordinate of the first endpoint of the second line
    *                                         segment.
    * @param lineSegmentStart2Y               the y-coordinate of the first endpoint of the second line
    *                                         segment.
    * @param lineSegmentStart2Z               the z-coordinate of the first endpoint of the second line
    *                                         segment.
    * @param lineSegmentEnd2X                 the x-coordinate of the second endpoint of the second
    *                                         line segment.
    * @param lineSegmentEnd2Y                 the y-coordinate of the second endpoint of the second
    *                                         line segment.
    * @param lineSegmentEnd2Z                 the z-coordinate of the second endpoint of the second
    *                                         line segment.
    * @param closestPointOnLineSegment1ToPack the 3D coordinates of the point P are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 3D coordinates of the point Q are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    */
   public static double closestPoint3DsBetweenTwoLineSegment3Ds(double lineSegmentStart1X,
                                                                double lineSegmentStart1Y,
                                                                double lineSegmentStart1Z,
                                                                double lineSegmentEnd1X,
                                                                double lineSegmentEnd1Y,
                                                                double lineSegmentEnd1Z,
                                                                double lineSegmentStart2X,
                                                                double lineSegmentStart2Y,
                                                                double lineSegmentStart2Z,
                                                                double lineSegmentEnd2X,
                                                                double lineSegmentEnd2Y,
                                                                double lineSegmentEnd2Z,
                                                                Point3DBasics closestPointOnLineSegment1ToPack,
                                                                Point3DBasics closestPointOnLineSegment2ToPack)
   {
      // Switching to the notation used in http://geomalgorithms.com/a07-_distance.html.
      // The line1 is defined by (P0, u) and the line2 by (Q0, v).
      double P0X = lineSegmentStart1X;
      double P0Y = lineSegmentStart1Y;
      double P0Z = lineSegmentStart1Z;
      double ux = lineSegmentEnd1X - lineSegmentStart1X;
      double uy = lineSegmentEnd1Y - lineSegmentStart1Y;
      double uz = lineSegmentEnd1Z - lineSegmentStart1Z;
      double Q0X = lineSegmentStart2X;
      double Q0Y = lineSegmentStart2Y;
      double Q0Z = lineSegmentStart2Z;
      double vx = lineSegmentEnd2X - lineSegmentStart2X;
      double vy = lineSegmentEnd2Y - lineSegmentStart2Y;
      double vz = lineSegmentEnd2Z - lineSegmentStart2Z;

      Point3DBasics Psc = closestPointOnLineSegment1ToPack;
      Point3DBasics Qtc = closestPointOnLineSegment2ToPack;

      double w0X = P0X - Q0X;
      double w0Y = P0Y - Q0Y;
      double w0Z = P0Z - Q0Z;

      double a = ux * ux + uy * uy + uz * uz;
      double b = ux * vx + uy * vy + uz * vz;
      double c = vx * vx + vy * vy + vz * vz;
      double d = ux * w0X + uy * w0Y + uz * w0Z;
      double e = vx * w0X + vy * w0Y + vz * w0Z;

      double ac = a * c;
      double bb = b * b;

      double delta = ac - bb;

      double sc, sNumerator, sDenominator = delta;
      double tc, tNumerator, tDenominator = delta;

      // check to see if the lines are parallel
      if (delta <= ONE_TRILLIONTH * Math.max(ac, bb))
      {
         /*
          * The lines are parallel, there's an infinite number of pairs, but for one chosen point on one of
          * the lines, there's only one closest point to it on the other line. So let's choose arbitrarily a
          * point on the lineSegment1 and calculate the point that is closest to it on the lineSegment2.
          */
         sNumerator = 0.0;
         sDenominator = 1.0;
         tNumerator = e;
         tDenominator = c;
      }
      else
      {
         sNumerator = b * e - c * d;
         tNumerator = a * e - b * d;

         if (sNumerator < 0.0)
         {
            sNumerator = 0.0;
            tNumerator = e;
            tDenominator = c;
         }
         else if (sNumerator > sDenominator)
         {
            sNumerator = sDenominator;
            tNumerator = e + b;
            tDenominator = c;
         }
      }

      if (tNumerator < 0.0)
      {
         tNumerator = 0.0;
         sNumerator = -d;
         if (sNumerator < 0.0)
            sNumerator = 0.0;
         else if (sNumerator > a)
            sNumerator = a;
         sDenominator = a;
      }
      else if (tNumerator > tDenominator)
      {
         tNumerator = tDenominator;
         sNumerator = -d + b;
         if (sNumerator < 0.0)
            sNumerator = 0.0;
         else if (sNumerator > a)
            sNumerator = a;
         sDenominator = a;
      }

      sc = Math.abs(sNumerator) < ONE_TRILLIONTH ? 0.0 : sNumerator / sDenominator;
      tc = Math.abs(tNumerator) < ONE_TRILLIONTH ? 0.0 : tNumerator / tDenominator;

      double PscX = sc * ux + P0X;
      double PscY = sc * uy + P0Y;
      double PscZ = sc * uz + P0Z;

      double QtcX = tc * vx + Q0X;
      double QtcY = tc * vy + Q0Y;
      double QtcZ = tc * vz + Q0Z;

      if (Psc != null)
         Psc.set(PscX, PscY, PscZ);
      if (Qtc != null)
         Qtc.set(QtcX, QtcY, QtcZ);

      double dx = PscX - QtcX;
      double dy = PscY - QtcY;
      double dz = PscZ - QtcZ;
      return EuclidCoreTools.norm(dx, dy, dz);
   }

   /**
    * Computes the area of a triangle defined by its three vertices: a, b, and c. No specific ordering
    * of the vertices is required.
    *
    * @param a first vertex of the triangle. Not modified.
    * @param b second vertex of the triangle. Not modified.
    * @param c third vertex of the triangle. Not modified.
    * @return the area of the triangle.
    */
   public static double triangleArea(Point2DReadOnly a, Point2DReadOnly b, Point2DReadOnly c)
   {
      return Math.abs(0.5 * (a.getX() * (b.getY() - c.getY()) + b.getX() * (c.getY() - a.getY()) + c.getX() * (a.getY() - b.getY())));
   }

   /**
    * Computes the area of a triangle defined by its three vertices: a, b, and c. No specific ordering
    * of the vertices is required.
    * <p>
    * This method uses {@link #triangleAreaHeron2(double, double, double)}.
    * </p>
    *
    * @param a first vertex of the triangle. Not modified.
    * @param b second vertex of the triangle. Not modified.
    * @param c third vertex of the triangle. Not modified.
    * @return the area of the triangle.
    */
   public static double triangleArea(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      return triangleAreaHeron2(a.distanceSquared(b), b.distanceSquared(c), c.distanceSquared(a));
   }

   /**
    * Computes the area of a triangle defined by the length of each of its side using Heron's formula:
    *
    * <pre>
    * s = 0.5(a + b + c)
    * A = &radic;{s(s - a)(s - b)(s - c)}
    * </pre>
    *
    * No specific ordering of the arguments is required.
    * <p>
    * Note that the method {@link #triangleArea(Point2DReadOnly, Point2DReadOnly, Point2DReadOnly)} is
    * computationally cheaper.
    * </p>
    *
    * @param lengthA the length of the first side.
    * @param lengthB the length of the second side.
    * @param lengthC the length of the third side.
    * @return the area of the triangle.
    */
   public static double triangleAreaHeron1(double lengthA, double lengthB, double lengthC)
   {
      double s = 0.5 * (lengthA + lengthB + lengthC);
      return EuclidCoreTools.squareRoot(s * (s - lengthA) * (s - lengthB) * (s - lengthC));
   }

   /**
    * Computes the area of a triangle defined by the length <b>squared</b> of each of its side using a
    * derivation from Heron's formula:
    *
    * <pre>
    * A = 0.25 * &radic;{4(a<sup>2</sup>b<sup>2</sup> + a<sup>2</sup>c<sup>2</sup> + b<sup>2</sup>c<sup>2</sup>) - (a<sup>2</sup> + b<sup>2</sup> + c<sup>2</sup>)<sup>2</sup>}
    * </pre>
    *
    * No specific ordering of the arguments is required.
    * <p>
    * Note that the method {@link #triangleArea(Point2DReadOnly, Point2DReadOnly, Point2DReadOnly)} is
    * computationally cheaper.
    * </p>
    *
    * @param lengthSquaredA the square of the length of the first side.
    * @param lengthSquaredB the square of the length of the second side.
    * @param lengthSquaredC the square of the length of the third side.
    * @return the area of the triangle.
    */
   public static double triangleAreaHeron2(double lengthSquaredA, double lengthSquaredB, double lengthSquaredC)
   {
      double areaPt1 = lengthSquaredA * lengthSquaredB;
      areaPt1 += lengthSquaredA * lengthSquaredC;
      areaPt1 += lengthSquaredB * lengthSquaredC;
      double areaPt2 = lengthSquaredA + lengthSquaredB + lengthSquaredC;
      double areaPt3 = 4.0 * areaPt1 - areaPt2 * areaPt2;
      return areaPt3 <= 0.0 ? 0.0 : 0.25 * EuclidCoreTools.squareRoot(areaPt3);
   }

   /**
    * Computes the coordinates of the center of the circumscribed circle of the triangle ABC.
    * <p>
    * Edge-case, if the problem is degenerate, i.e. the three points are on a line or all equal, this
    * method fails and returns {@code false}.
    * </p>
    * <p>
    * Algorithm from
    * <a href="https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates">Wikipedia
    * article</a>.
    * </p>
    *
    * @param A                  the position of the first vertex of the triangle. Not modified.
    * @param B                  the position of the second vertex of the triangle. Not modified.
    * @param C                  the position of the third vertex of the triangle. Not modified.
    * @param circumcenterToPack the coordinates of the circumscribed circle's center.
    * @return {@code true} if the calculation was successful, {@code false} otherwise.
    */
   public static boolean triangleCircumcenter(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DBasics circumcenterToPack)
   {
      double ASquared = A.distanceFromOriginSquared();
      double BSquared = B.distanceFromOriginSquared();
      double CSquared = C.distanceFromOriginSquared();

      double ByCy = B.getY() - C.getY();
      double CxBx = C.getX() - B.getX();

      double a = 0.5 / (A.getX() * ByCy + A.getY() * CxBx + B.getX() * C.getY() - B.getY() * C.getX());

      if (!Double.isFinite(a))
         return false;

      double CSquaredBSquared = CSquared - BSquared;
      double sx = ASquared * ByCy + A.getY() * CSquaredBSquared + BSquared * C.getY() - B.getY() * CSquared;
      double sy = -A.getX() * CSquaredBSquared + ASquared * CxBx + B.getX() * CSquared - BSquared * C.getX();

      circumcenterToPack.set(sx, sy);
      circumcenterToPack.scale(a);
      return true;
   }

   /**
    * Computes the coordinates of the center of the circumscribed circle of the triangle ABC.
    * <p>
    * Edge-case, if the problem is degenerate, i.e. the three points are on a line or all equal, this
    * method fails and returns {@code false}.
    * </p>
    * <p>
    * Algorithm from <a href=
    * "https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products">Wikipedia
    * article</a>.
    * </p>
    *
    * @param A                  the position of the first vertex of the triangle. Not modified.
    * @param B                  the position of the second vertex of the triangle. Not modified.
    * @param C                  the position of the third vertex of the triangle. Not modified.
    * @param circumcenterToPack the coordinates of the circumscribed circle's center.
    * @return {@code true} if the calculation was successful, {@code false} otherwise.
    */
   public static boolean triangleCircumcenter(Point3DReadOnly A, Point3DReadOnly B, Point3DReadOnly C, Point3DBasics circumcenterToPack)
   {
      double ux = B.getX() - C.getX();
      double uy = B.getY() - C.getY();
      double uz = B.getZ() - C.getZ();

      double vx = A.getX() - C.getX();
      double vy = A.getY() - C.getY();
      double vz = A.getZ() - C.getZ();

      double wx = A.getX() - B.getX();
      double wy = A.getY() - B.getY();
      double wz = A.getZ() - B.getZ();

      double uSquared = EuclidCoreTools.normSquared(ux, uy, uz);
      double vSquared = EuclidCoreTools.normSquared(vx, vy, vz);
      double wSquared = EuclidCoreTools.normSquared(wx, wy, wz);
      double uv = ux * vx + uy * vy + uz * vz;
      double uw = ux * wx + uy * wy + uz * wz;
      double vw = vx * wx + vy * wy + vz * wz;

      double ucwx = wy * uz - wz * uy;
      double ucwy = wz * ux - wx * uz;
      double ucwz = wx * uy - wy * ux;
      double ucwSquared = 0.5 / EuclidCoreTools.normSquared(ucwx, ucwy, ucwz);

      if (!Double.isFinite(ucwSquared))
         return false;

      double alpha = uSquared * vw * ucwSquared;
      double beta = -vSquared * uw * ucwSquared;
      double gamma = wSquared * uv * ucwSquared;

      double px = alpha * A.getX() + beta * B.getX() + gamma * C.getX();
      double py = alpha * A.getY() + beta * B.getY() + gamma * C.getY();
      double pz = alpha * A.getZ() + beta * B.getZ() + gamma * C.getZ();
      circumcenterToPack.set(px, py, pz);

      return true;
   }

   /**
    * Calculates the distance between two points.
    *
    * @param firstPointX  the x-coordinate of the first point.
    * @param firstPointY  the y-coordinate of the first point.
    * @param secondPointX the x-coordinate of the second point.
    * @param secondPointY the y-coordinate of the second point.
    * @return the distance between the two points.
    */
   public static double distanceBetweenPoint2Ds(double firstPointX, double firstPointY, double secondPointX, double secondPointY)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredBetweenPoint2Ds(firstPointX, firstPointY, secondPointX, secondPointY));
   }

   /**
    * Calculates the distance between two points.
    *
    * @param firstPointX the x-coordinate of the first point.
    * @param firstPointY the y-coordinate of the first point.
    * @param secondPoint the coordinates of the second point. Not modified.
    * @return the distance between the two points.
    */
   public static double distanceBetweenPoint2Ds(double firstPointX, double firstPointY, Point2DReadOnly secondPoint)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredBetweenPoint2Ds(firstPointX, firstPointY, secondPoint));
   }

   /**
    * Calculates the square value of the distance between two points.
    *
    * @param firstPointX  the x-coordinate of the first point.
    * @param firstPointY  the y-coordinate of the first point.
    * @param secondPointX the x-coordinate of the second point.
    * @param secondPointY the y-coordinate of the second point.
    * @return the distance squared between the two points.
    */
   public static double distanceSquaredBetweenPoint2Ds(double firstPointX, double firstPointY, double secondPointX, double secondPointY)
   {
      double deltaX = secondPointX - firstPointX;
      double deltaY = secondPointY - firstPointY;
      return EuclidCoreTools.normSquared(deltaX, deltaY);
   }

   /**
    * Calculates the square value of the distance between two points.
    *
    * @param firstPointX the x-coordinate of the first point.
    * @param firstPointY the y-coordinate of the first point.
    * @param secondPoint the coordinates of the second point. Not modified.
    * @return the distance squared between the two points.
    */
   public static double distanceSquaredBetweenPoint2Ds(double firstPointX, double firstPointY, Point2DReadOnly secondPoint)
   {
      double deltaX = secondPoint.getX() - firstPointX;
      double deltaY = secondPoint.getY() - firstPointY;
      return EuclidCoreTools.normSquared(deltaX, deltaY);
   }

   /**
    * Calculates the distance between two points.
    *
    * @param firstPointX  the x-coordinate of the first point.
    * @param firstPointY  the y-coordinate of the first point.
    * @param firstPointZ  the z-coordinate of the first point.
    * @param secondPointX the x-coordinate of the second point.
    * @param secondPointY the y-coordinate of the second point.
    * @param secondPointZ the z-coordinate of the second point.
    * @return the distance between the two points.
    */
   public static double distanceBetweenPoint3Ds(double firstPointX,
                                                double firstPointY,
                                                double firstPointZ,
                                                double secondPointX,
                                                double secondPointY,
                                                double secondPointZ)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredBetweenPoint3Ds(firstPointX, firstPointY, firstPointZ, secondPointX, secondPointY, secondPointZ));
   }

   /**
    * Calculates the distance between two points.
    *
    * @param firstPointX the x-coordinate of the first point.
    * @param firstPointY the y-coordinate of the first point.
    * @param firstPointZ the z-coordinate of the first point.
    * @param secondPoint the coordinates of the second point. Not modified.
    * @return the distance between the two points.
    */
   public static double distanceBetweenPoint3Ds(double firstPointX, double firstPointY, double firstPointZ, Point3DReadOnly secondPoint)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredBetweenPoint3Ds(firstPointX, firstPointY, firstPointZ, secondPoint));
   }

   /**
    * Calculates the square value of the distance between two points.
    *
    * @param firstPointX  the x-coordinate of the first point.
    * @param firstPointY  the y-coordinate of the first point.
    * @param firstPointZ  the z-coordinate of the first point.
    * @param secondPointX the x-coordinate of the second point.
    * @param secondPointY the y-coordinate of the second point.
    * @param secondPointZ the z-coordinate of the second point.
    * @return the distance squared between the two points.
    */
   public static double distanceSquaredBetweenPoint3Ds(double firstPointX,
                                                       double firstPointY,
                                                       double firstPointZ,
                                                       double secondPointX,
                                                       double secondPointY,
                                                       double secondPointZ)
   {
      double deltaX = secondPointX - firstPointX;
      double deltaY = secondPointY - firstPointY;
      double deltaZ = secondPointZ - firstPointZ;
      return EuclidCoreTools.normSquared(deltaX, deltaY, deltaZ);
   }

   /**
    * Calculates the square value of the distance between two points.
    *
    * @param firstPointX the x-coordinate of the first point.
    * @param firstPointY the y-coordinate of the first point.
    * @param firstPointZ the z-coordinate of the first point.
    * @param secondPoint the coordinates of the second point. Not modified.
    * @return the distance squared between the two points.
    */
   public static double distanceSquaredBetweenPoint3Ds(double firstPointX, double firstPointY, double firstPointZ, Point3DReadOnly secondPoint)
   {
      double deltaX = secondPoint.getX() - firstPointX;
      double deltaY = secondPoint.getY() - firstPointY;
      double deltaZ = secondPoint.getZ() - firstPointZ;
      return EuclidCoreTools.normSquared(deltaX, deltaY, deltaZ);
   }

   /**
    * This methods computes the minimum distance between the two infinitely long 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param pointOnLine1   a 3D point on the first line. Not modified.
    * @param lineDirection1 the 3D direction of the first line. Not modified.
    * @param pointOnLine2   a 3D point on the second line. Not modified.
    * @param lineDirection2 the 3D direction of the second line. Not modified.
    * @return the minimum distance between the two lines.
    */
   public static double distanceBetweenTwoLine3Ds(Point3DReadOnly pointOnLine1,
                                                  Vector3DReadOnly lineDirection1,
                                                  Point3DReadOnly pointOnLine2,
                                                  Vector3DReadOnly lineDirection2)
   {
      return closestPoint3DsBetweenTwoLine3Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, null, null);
   }

   /**
    * This methods computes the minimum distance between the two 2D line segments with finite length.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the minimum distance between the two line segments.
    */
   public static double distanceBetweenTwoLineSegment2Ds(Point2DReadOnly lineSegmentStart1,
                                                         Point2DReadOnly lineSegmentEnd1,
                                                         Point2DReadOnly lineSegmentStart2,
                                                         Point2DReadOnly lineSegmentEnd2)
   {
      return closestPoint2DsBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, null, null);
   }

   /**
    * This methods computes the minimum distance between the two 3D line segments with finite length.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the minimum distance between the two line segments.
    */
   public static double distanceBetweenTwoLineSegment3Ds(Point3DReadOnly lineSegmentStart1,
                                                         Point3DReadOnly lineSegmentEnd1,
                                                         Point3DReadOnly lineSegmentStart2,
                                                         Point3DReadOnly lineSegmentEnd2)
   {
      return closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, null, null);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by a point
    * and a direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX         x-coordinate of the query.
    * @param pointY         y-coordinate of the query.
    * @param pointOnLineX   x-coordinate of a point located on the line.
    * @param pointOnLineY   y-coordinate of a point located on the line.
    * @param lineDirectionX x-component of the line direction.
    * @param lineDirectionY y-component of the line direction.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToLine2D(double pointX,
                                                    double pointY,
                                                    double pointOnLineX,
                                                    double pointOnLineY,
                                                    double lineDirectionX,
                                                    double lineDirectionY)
   {
      return distanceFromPoint2DToLine2D(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, false);
   }

   private static double distanceFromPoint2DToLine2D(double pointX,
                                                     double pointY,
                                                     double pointOnLineX,
                                                     double pointOnLineY,
                                                     double lineDirectionX,
                                                     double lineDirectionY,
                                                     boolean isDirectionUnitary)
   {
      return Math.abs(signedDistanceFromPoint2DToLine2D(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, isDirectionUnitary));
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by two
    * points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code firstPointOnLine.distance(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x-coordinate of the query.
    * @param pointY            y-coordinate of the query.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToLine2D(double pointX, double pointY, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      return distanceFromPoint2DToLine2D(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by a point
    * and a direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX        x-coordinate of the query.
    * @param pointY        y-coordinate of the query.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToLine2D(double pointX, double pointY, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return distanceFromPoint2DToLine2D(pointX,
                                         pointY,
                                         pointOnLine.getX(),
                                         pointOnLine.getY(),
                                         lineDirection.getX(),
                                         lineDirection.getY(),
                                         lineDirection instanceof UnitVector2DReadOnly);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by two
    * points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code firstPointOnLine.distance(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point             2D point to compute the distance from the line. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToLine2D(Point2DReadOnly point, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      return distanceFromPoint2DToLine2D(point.getX(), point.getY(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by a point
    * and a direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point         2D point to compute the distance from the line. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToLine2D(Point2DReadOnly point, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return distanceFromPoint2DToLine2D(point.getX(), point.getY(), pointOnLine, lineDirection);
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x coordinate of point to be tested.
    * @param pointY            y coordinate of point to be tested.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @return the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceFromPoint2DToLineSegment2D(double pointX,
                                                           double pointY,
                                                           double lineSegmentStartX,
                                                           double lineSegmentStartY,
                                                           double lineSegmentEndX,
                                                           double lineSegmentEndY)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredFromPoint2DToLineSegment2D(pointX,
                                                                                  pointY,
                                                                                  lineSegmentStartX,
                                                                                  lineSegmentStartY,
                                                                                  lineSegmentEndX,
                                                                                  lineSegmentEndY));
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x coordinate of point to be tested.
    * @param pointY           y coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceFromPoint2DToLineSegment2D(double pointX, double pointY, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return distanceFromPoint2DToLineSegment2D(pointX, pointY, lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            2D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceFromPoint2DToLineSegment2D(Point2DReadOnly point, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return distanceFromPoint2DToLineSegment2D(point.getX(), point.getY(), lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the minimum distance between a 2D point and a 2D ray defined by its origin and a
    * direction.
    * <p>
    * When the query is located in front of the ray, this is equivalent to calculating the distance
    * from the query to the line that is collinear with the ray. When the query is located behind the
    * ray's origin, this is equivalent to calculating the distance between the query and the origin of
    * the ray.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code rayDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the distance
    * between {@code rayOrigin} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX        x-coordinate of the query.
    * @param pointY        y-coordinate of the query.
    * @param rayOriginX    x-coordinate of the ray origin.
    * @param rayOriginY    y-coordinate of the ray origin.
    * @param rayDirectionX x-component of the ray direction.
    * @param rayDirectionY y-component of the ray direction.
    * @return the minimum distance between the 2D point and the 2D ray.
    */
   public static double distanceFromPoint2DToRay2D(double pointX,
                                                   double pointY,
                                                   double rayOriginX,
                                                   double rayOriginY,
                                                   double rayDirectionX,
                                                   double rayDirectionY)
   {
      return distanceFromPoint2DToRay2D(pointX, pointY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY, false);
   }

   private static double distanceFromPoint2DToRay2D(double pointX,
                                                    double pointY,
                                                    double rayOriginX,
                                                    double rayOriginY,
                                                    double rayDirectionX,
                                                    double rayDirectionY,
                                                    boolean isDirectionUnitary)
   {
      if (isPoint2DInFrontOfRay2D(pointX, pointY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
         return Math.abs(signedDistanceFromPoint2DToLine2D(pointX, pointY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY, isDirectionUnitary));
      else
         return distanceBetweenPoint2Ds(pointX, pointY, rayOriginX, rayOriginY);
   }

   /**
    * Returns the minimum distance between a 2D point and a 2D ray defined by its origin and a
    * direction.
    * <p>
    * When the query is located in front of the ray, this is equivalent to calculating the distance
    * from the query to the line that is collinear with the ray. When the query is located behind the
    * ray's origin, this is equivalent to calculating the distance between the query and the origin of
    * the ray.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code rayDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the distance
    * between {@code rayOrigin} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX       x-coordinate of the query.
    * @param pointY       y-coordinate of the query.
    * @param rayOrigin    a point located on the line. Not modified.
    * @param rayDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToRay2D(double pointX, double pointY, Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection)
   {
      return distanceFromPoint2DToRay2D(pointX,
                                        pointY,
                                        rayOrigin.getX(),
                                        rayOrigin.getY(),
                                        rayDirection.getX(),
                                        rayDirection.getY(),
                                        rayDirection instanceof UnitVector2DReadOnly);
   }

   /**
    * Returns the minimum distance between a 2D point and a 2D ray defined by its origin and a
    * direction.
    * <p>
    * When the query is located in front of the ray, this is equivalent to calculating the distance
    * from the query to the line that is collinear with the ray. When the query is located behind the
    * ray's origin, this is equivalent to calculating the distance between the query and the origin of
    * the ray.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code rayDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the distance
    * between {@code rayOrigin} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point        the coordinates of the query.
    * @param rayOrigin    a point located on the line. Not modified.
    * @param rayDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPoint2DToRay2D(Point2DReadOnly point, Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection)
   {
      return distanceFromPoint2DToRay2D(point.getX(), point.getY(), rayOrigin, rayDirection);
   }

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by a
    * point and a direction.
    * <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX         x-coordinate of the 3D point to compute the distance from the line. Not
    *                       modified.
    * @param pointY         y-coordinate of the 3D point to compute the distance from the line. Not
    *                       modified.
    * @param pointZ         z-coordinate of the 3D point to compute the distance from the line. Not
    *                       modified.
    * @param pointOnLineX   x-coordinate of a point located on the line.
    * @param pointOnLineY   y-coordinate of a point located on the line.
    * @param pointOnLineZ   z-coordinate of a point located on the line.
    * @param lineDirectionX x-component of the line direction.
    * @param lineDirectionY y-component of the line direction.
    * @param lineDirectionZ z-component of the line direction.
    * @return the minimum distance between the 3D point and the 3D line.
    */
   public static double distanceFromPoint3DToLine3D(double pointX,
                                                    double pointY,
                                                    double pointZ,
                                                    double pointOnLineX,
                                                    double pointOnLineY,
                                                    double pointOnLineZ,
                                                    double lineDirectionX,
                                                    double lineDirectionY,
                                                    double lineDirectionZ)
   {
      return distanceFromPoint3DToLine3D(pointX,
                                         pointY,
                                         pointZ,
                                         pointOnLineX,
                                         pointOnLineY,
                                         pointOnLineZ,
                                         lineDirectionX,
                                         lineDirectionY,
                                         lineDirectionZ,
                                         false);
   }

   private static double distanceFromPoint3DToLine3D(double pointX,
                                                     double pointY,
                                                     double pointZ,
                                                     double pointOnLineX,
                                                     double pointOnLineY,
                                                     double pointOnLineZ,
                                                     double lineDirectionX,
                                                     double lineDirectionY,
                                                     double lineDirectionZ,
                                                     boolean isDirectionUnitary)
   {
      double directionMagnitude = isDirectionUnitary ? 1.0 : EuclidCoreTools.fastNorm(lineDirectionX, lineDirectionY, lineDirectionZ);

      double dx = pointOnLineX - pointX;
      double dy = pointOnLineY - pointY;
      double dz = pointOnLineZ - pointZ;

      if (directionMagnitude < ONE_TRILLIONTH)
      {
         return EuclidCoreTools.norm(dx, dy, dz);
      }
      else
      {
         double crossX = lineDirectionY * dz - lineDirectionZ * dy;
         double crossY = lineDirectionZ * dx - lineDirectionX * dz;
         double crossZ = lineDirectionX * dy - lineDirectionY * dx;
         double distance = EuclidCoreTools.norm(crossX, crossY, crossZ);
         distance /= directionMagnitude;

         return distance;
      }
   }

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by two
    * points. <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful
    * link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code firstPointOnLine.distance(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point             3D point to compute the distance from the line. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 3D point and the 3D line.
    */
   public static double distanceFromPoint3DToLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double pointOnLineZ = firstPointOnLine.getZ();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double lineDirectionZ = secondPointOnLine.getZ() - firstPointOnLine.getZ();
      return distanceFromPoint3DToLine3D(point.getX(),
                                         point.getY(),
                                         point.getZ(),
                                         pointOnLineX,
                                         pointOnLineY,
                                         pointOnLineZ,
                                         lineDirectionX,
                                         lineDirectionY,
                                         lineDirectionZ);
   }

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by a
    * point and a direction.
    * <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point         3D point to compute the distance from the line. Not modified.
    * @param pointOnLine   point located on the line. Not modified.
    * @param lineDirection direction of the line. Not modified.
    * @return the minimum distance between the 3D point and the 3D line.
    */
   public static double distanceFromPoint3DToLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return distanceFromPoint3DToLine3D(point.getX(),
                                         point.getY(),
                                         point.getZ(),
                                         pointOnLine.getX(),
                                         pointOnLine.getY(),
                                         pointOnLine.getZ(),
                                         lineDirection.getX(),
                                         lineDirection.getY(),
                                         lineDirection.getZ());
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x-coordinate of point to be tested.
    * @param pointY            y-coordinate of point to be tested.
    * @param pointZ            z-coordinate of point to be tested.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentStartZ the z-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @param lineSegmentEndZ   the z-coordinate of the line segment second endpoint.
    * @return the minimum distance between the 3D point and the 3D line segment.
    */
   public static double distanceFromPoint3DToLineSegment3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           double lineSegmentStartX,
                                                           double lineSegmentStartY,
                                                           double lineSegmentStartZ,
                                                           double lineSegmentEndX,
                                                           double lineSegmentEndY,
                                                           double lineSegmentEndZ)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredFromPoint3DToLineSegment3D(pointX,
                                                                                  pointY,
                                                                                  pointZ,
                                                                                  lineSegmentStartX,
                                                                                  lineSegmentStartY,
                                                                                  lineSegmentStartZ,
                                                                                  lineSegmentEndX,
                                                                                  lineSegmentEndY,
                                                                                  lineSegmentEndZ));
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x-coordinate of point to be tested.
    * @param pointY           y-coordinate of point to be tested.
    * @param pointZ           z-coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 3D point and the 3D line segment.
    */
   public static double distanceFromPoint3DToLineSegment3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           Point3DReadOnly lineSegmentStart,
                                                           Point3DReadOnly lineSegmentEnd)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredFromPoint3DToLineSegment3D(pointX, pointY, pointZ, lineSegmentStart, lineSegmentEnd));
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            3D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 3D point and the 3D line segment.
    */
   public static double distanceFromPoint3DToLineSegment3D(Point3DReadOnly point, Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      return distanceFromPoint3DToLineSegment3D(point.getX(), point.getY(), point.getZ(), lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes the minimum distance between a given point and a plane.
    *
    * @param pointX       the x-coordinate of the query. Not modified.
    * @param pointY       the y-coordinate of the query. Not modified.
    * @param pointZ       the z-coordinate of the query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the distance between the point and the plane.
    */
   public static double distanceFromPoint3DToPlane3D(double pointX, double pointY, double pointZ, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return Math.abs(signedDistanceFromPoint3DToPlane3D(pointX, pointY, pointZ, pointOnPlane, planeNormal));
   }

   /**
    * Computes the minimum distance between a given point and a plane.
    *
    * @param point        the 3D query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the distance between the point and the plane.
    */
   public static double distanceFromPoint3DToPlane3D(Point3DReadOnly point, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return Math.abs(signedDistanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal));
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param pointX        the x-coordinate of the query. Not modified.
    * @param pointY        the y-coordinate of the query. Not modified.
    * @param pointZ        the z-coordinate of the query. Not modified.
    * @param pointOnPlaneX the x-coordinate of a point located on the plane. Not modified.
    * @param pointOnPlaneY the y-coordinate of a point located on the plane. Not modified.
    * @param pointOnPlaneZ the z-coordinate of a point located on the plane. Not modified.
    * @param planeNormalX  the x-component of the normal of the plane. Not modified.
    * @param planeNormalY  the y-component of the normal of the plane. Not modified.
    * @param planeNormalZ  the z-component of the normal of the plane. Not modified.
    * @return the signed distance between the point and the plane.
    */
   public static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           double pointOnPlaneX,
                                                           double pointOnPlaneY,
                                                           double pointOnPlaneZ,
                                                           double planeNormalX,
                                                           double planeNormalY,
                                                           double planeNormalZ)
   {
      return signedDistanceFromPoint3DToPlane3D(pointX,
                                                pointY,
                                                pointZ,
                                                pointOnPlaneX,
                                                pointOnPlaneY,
                                                pointOnPlaneZ,
                                                planeNormalX,
                                                planeNormalY,
                                                planeNormalZ,
                                                false);
   }

   private static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                            double pointY,
                                                            double pointZ,
                                                            double pointOnPlaneX,
                                                            double pointOnPlaneY,
                                                            double pointOnPlaneZ,
                                                            double planeNormalX,
                                                            double planeNormalY,
                                                            double planeNormalZ,
                                                            boolean isNormalUnitary)
   {
      double dx = pointX - pointOnPlaneX;
      double dy = pointY - pointOnPlaneY;
      double dz = pointZ - pointOnPlaneZ;

      if (isNormalUnitary)
      {
         return dx * planeNormalX + dy * planeNormalY + dz * planeNormalZ;
      }
      else
      {
         double normalMagnitude = EuclidCoreTools.normSquared(planeNormalX, planeNormalY, planeNormalZ);

         if (normalMagnitude < ONE_TRILLIONTH)
            return EuclidCoreTools.norm(dx, dy, dz);
         else
            return (dx * planeNormalX + dy * planeNormalY + dz * planeNormalZ) / EuclidCoreTools.squareRoot(normalMagnitude);
      }
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param pointX              the x-coordinate of the query. Not modified.
    * @param pointY              the y-coordinate of the query. Not modified.
    * @param pointZ              the z-coordinate of the query. Not modified.
    * @param pointOnPlaneX       the x-coordinate of a point located on the plane. Not modified.
    * @param pointOnPlaneY       the y-coordinate of a point located on the plane. Not modified.
    * @param pointOnPlaneZ       the z-coordinate of a point located on the plane. Not modified.
    * @param planeFirstTangentX  the x-component of a first tangent of the infinite plane.
    * @param planeFirstTangentY  the y-component of a first tangent of the infinite plane.
    * @param planeFirstTangentZ  the z-component of a first tangent of the infinite plane.
    * @param planeSecondTangentX the x-component of a second tangent of the infinite plane.
    * @param planeSecondTangentY the y-component of a second tangent of the infinite plane.
    * @param planeSecondTangentZ the z-component of a second tangent of the infinite plane.
    * @return the signed distance between the point and the plane.
    */
   public static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           double pointOnPlaneX,
                                                           double pointOnPlaneY,
                                                           double pointOnPlaneZ,
                                                           double planeFirstTangentX,
                                                           double planeFirstTangentY,
                                                           double planeFirstTangentZ,
                                                           double planeSecondTangentX,
                                                           double planeSecondTangentY,
                                                           double planeSecondTangentZ)
   {
      double planeNormalX = planeFirstTangentY * planeSecondTangentZ - planeFirstTangentZ * planeSecondTangentY;
      double planeNormalY = planeFirstTangentZ * planeSecondTangentX - planeFirstTangentX * planeSecondTangentZ;
      double planeNormalZ = planeFirstTangentX * planeSecondTangentY - planeFirstTangentY * planeSecondTangentX;
      return signedDistanceFromPoint3DToPlane3D(pointX, pointY, pointZ, pointOnPlaneX, pointOnPlaneY, pointOnPlaneZ, planeNormalX, planeNormalY, planeNormalZ);

   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param pointX       the x-coordinate of the query. Not modified.
    * @param pointY       the y-coordinate of the query. Not modified.
    * @param pointZ       the z-coordinate of the query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the signed distance between the point and the plane.
    */
   public static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           Point3DReadOnly pointOnPlane,
                                                           Vector3DReadOnly planeNormal)
   {
      return signedDistanceFromPoint3DToPlane3D(pointX,
                                                pointY,
                                                pointZ,
                                                pointOnPlane.getX(),
                                                pointOnPlane.getY(),
                                                pointOnPlane.getZ(),
                                                planeNormal.getX(),
                                                planeNormal.getY(),
                                                planeNormal.getZ(),
                                                planeNormal instanceof UnitVector3DReadOnly);
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param point        the query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the signed distance between the point and the plane.
    */
   public static double signedDistanceFromPoint3DToPlane3D(Point3DReadOnly point, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return signedDistanceFromPoint3DToPlane3D(point.getX(), point.getY(), point.getZ(), pointOnPlane, planeNormal);
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param pointX             the x-coordinate of the query. Not modified.
    * @param pointY             the y-coordinate of the query. Not modified.
    * @param pointZ             the z-coordinate of the query. Not modified.
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return the signed distance between the point and the plane.
    */
   public static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           Point3DReadOnly pointOnPlane,
                                                           Vector3DReadOnly planeFirstTangent,
                                                           Vector3DReadOnly planeSecondTangent)
   {
      return signedDistanceFromPoint3DToPlane3D(pointX,
                                                pointY,
                                                pointZ,
                                                pointOnPlane.getX(),
                                                pointOnPlane.getY(),
                                                pointOnPlane.getZ(),
                                                planeFirstTangent.getX(),
                                                planeFirstTangent.getY(),
                                                planeFirstTangent.getZ(),
                                                planeSecondTangent.getX(),
                                                planeSecondTangent.getY(),
                                                planeSecondTangent.getZ());
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param point              the query. Not modified.
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return the signed distance between the point and the plane.
    */
   public static double signedDistanceFromPoint3DToPlane3D(Point3DReadOnly point,
                                                           Point3DReadOnly pointOnPlane,
                                                           Vector3DReadOnly planeFirstTangent,
                                                           Vector3DReadOnly planeSecondTangent)
   {
      return signedDistanceFromPoint3DToPlane3D(point.getX(), point.getY(), point.getZ(), pointOnPlane, planeFirstTangent, planeSecondTangent);
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x coordinate of point to be tested.
    * @param pointY            y coordinate of point to be tested.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @return the square of the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceSquaredFromPoint2DToLineSegment2D(double pointX,
                                                                  double pointY,
                                                                  double lineSegmentStartX,
                                                                  double lineSegmentStartY,
                                                                  double lineSegmentEndX,
                                                                  double lineSegmentEndY)
   {
      double percentage = percentageAlongLineSegment2D(pointX, pointY, lineSegmentStartX, lineSegmentStartY, lineSegmentEndX, lineSegmentEndY);

      if (percentage > 1.0)
         percentage = 1.0;
      else if (percentage < 0.0)
         percentage = 0.0;

      double projectionX = (1.0 - percentage) * lineSegmentStartX + percentage * lineSegmentEndX;
      double projectionY = (1.0 - percentage) * lineSegmentStartY + percentage * lineSegmentEndY;

      double dx = projectionX - pointX;
      double dy = projectionY - pointY;
      return dx * dx + dy * dy;
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x coordinate of point to be tested.
    * @param pointY           y coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceSquaredFromPoint2DToLineSegment2D(double pointX,
                                                                  double pointY,
                                                                  Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd)
   {
      return distanceSquaredFromPoint2DToLineSegment2D(pointX,
                                                       pointY,
                                                       lineSegmentStart.getX(),
                                                       lineSegmentStart.getY(),
                                                       lineSegmentEnd.getX(),
                                                       lineSegmentEnd.getY());
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            coordinates of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceSquaredFromPoint2DToLineSegment2D(Point2DReadOnly point, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return distanceSquaredFromPoint2DToLineSegment2D(point.getX(),
                                                       point.getY(),
                                                       lineSegmentStart.getX(),
                                                       lineSegmentStart.getY(),
                                                       lineSegmentEnd.getX(),
                                                       lineSegmentEnd.getY());
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x-coordinate of point to be tested.
    * @param pointY           y-coordinate of point to be tested.
    * @param pointZ           z-coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 3D point and the 3D line segment.
    */
   public static double distanceSquaredFromPoint3DToLineSegment3D(double pointX,
                                                                  double pointY,
                                                                  double pointZ,
                                                                  Point3DReadOnly lineSegmentStart,
                                                                  Point3DReadOnly lineSegmentEnd)
   {
      return distanceSquaredFromPoint3DToLineSegment3D(pointX,
                                                       pointY,
                                                       pointZ,
                                                       lineSegmentStart.getX(),
                                                       lineSegmentStart.getY(),
                                                       lineSegmentStart.getZ(),
                                                       lineSegmentEnd.getX(),
                                                       lineSegmentEnd.getY(),
                                                       lineSegmentEnd.getZ());
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x-coordinate of point to be tested.
    * @param pointY            y-coordinate of point to be tested.
    * @param pointZ            z-coordinate of point to be tested.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentStartZ the z-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @param lineSegmentEndZ   the z-coordinate of the line segment second endpoint.
    * @return the square of the minimum distance between the 3D point and the 3D line segment.
    */
   public static double distanceSquaredFromPoint3DToLineSegment3D(double pointX,
                                                                  double pointY,
                                                                  double pointZ,
                                                                  double lineSegmentStartX,
                                                                  double lineSegmentStartY,
                                                                  double lineSegmentStartZ,
                                                                  double lineSegmentEndX,
                                                                  double lineSegmentEndY,
                                                                  double lineSegmentEndZ)
   {
      double percentage = percentageAlongLineSegment3D(pointX,
                                                       pointY,
                                                       pointZ,
                                                       lineSegmentStartX,
                                                       lineSegmentStartY,
                                                       lineSegmentStartZ,
                                                       lineSegmentEndX,
                                                       lineSegmentEndY,
                                                       lineSegmentEndZ);

      if (percentage > 1.0)
         percentage = 1.0;
      else if (percentage < 0.0)
         percentage = 0.0;

      double oneMinusPercentage = 1.0 - percentage;
      double projectionX = oneMinusPercentage * lineSegmentStartX + percentage * lineSegmentEndX;
      double projectionY = oneMinusPercentage * lineSegmentStartY + percentage * lineSegmentEndY;
      double projectionZ = oneMinusPercentage * lineSegmentStartZ + percentage * lineSegmentEndZ;

      double dx = projectionX - pointX;
      double dy = projectionY - pointY;
      double dz = projectionZ - pointZ;
      return dx * dx + dy * dy + dz * dz;
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            3D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 3D point and the 3D line segment.
    */
   public static double distanceSquaredFromPoint3DToLineSegment3D(Point3DReadOnly point, Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      return distanceSquaredFromPoint3DToLineSegment3D(point.getX(), point.getY(), point.getZ(), lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Test if a given line segment intersects a given plane.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the line segment endpoints are equal, this method returns {@code false} whether the endpoints
    * are on the plane or not.
    * <li>one of the line segment endpoints is exactly on the plane, this method returns false.
    * </ul>
    * </p>
    *
    * @param pointOnPlane     a point located on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if an intersection line segment - plane exists, {@code false} otherwise.
    */
   public static boolean doesLineSegment3DIntersectPlane3D(Point3DReadOnly pointOnPlane,
                                                           Vector3DReadOnly planeNormal,
                                                           Point3DReadOnly lineSegmentStart,
                                                           Point3DReadOnly lineSegmentEnd)
   {
      double d, ansStart, ansEnd;

      d = -planeNormal.getX() * pointOnPlane.getX();
      d -= planeNormal.getY() * pointOnPlane.getY();
      d -= planeNormal.getZ() * pointOnPlane.getZ();

      ansStart = planeNormal.getX() * lineSegmentStart.getX();
      ansStart += planeNormal.getY() * lineSegmentStart.getY();
      ansStart += planeNormal.getZ() * lineSegmentStart.getZ();
      ansStart += d;

      ansEnd = planeNormal.getX() * lineSegmentEnd.getX();
      ansEnd += planeNormal.getY() * lineSegmentEnd.getY();
      ansEnd += planeNormal.getZ() * lineSegmentEnd.getZ();
      ansEnd += d;

      return ansStart * ansEnd < 0.0;
   }

   /**
    * Tests if an intersection exists between an infinitely long 2D line (defined by a 2D point and a
    * 2D direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param pointOnLineX     the x-coordinate of a point located on the line.
    * @param pointOnLineY     the y-coordinate of a point located on the line.
    * @param lineDirectionX   the x-component of the line direction.
    * @param lineDirectionY   the y-component of the line direction.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    */
   public static boolean doLine2DAndLineSegment2DIntersect(double pointOnLineX,
                                                           double pointOnLineY,
                                                           double lineDirectionX,
                                                           double lineDirectionY,
                                                           Point2DReadOnly lineSegmentStart,
                                                           Point2DReadOnly lineSegmentEnd)
   {
      return intersectionBetweenLine2DAndLineSegment2D(pointOnLineX,
                                                       pointOnLineY,
                                                       lineDirectionX,
                                                       lineDirectionY,
                                                       lineSegmentStart.getX(),
                                                       lineSegmentStart.getY(),
                                                       lineSegmentEnd.getX(),
                                                       lineSegmentEnd.getY(),
                                                       null);
   }

   /**
    * Tests if an intersection exists between an infinitely long 2D line (defined by a 2D point and a
    * 2D direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the line direction. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    */
   public static boolean doLine2DAndLineSegment2DIntersect(Point2DReadOnly pointOnLine,
                                                           Vector2DReadOnly lineDirection,
                                                           Point2DReadOnly lineSegmentStart,
                                                           Point2DReadOnly lineSegmentEnd)
   {
      return intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, null);
   }

   /**
    * Test if two line segments intersect each other.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, this method returns false.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart1x x-coordinate of the first endpoint of the first line segment.
    * @param lineSegmentStart1y y-coordinate of the first endpoint of the first line segment.
    * @param lineSegmentEnd1x   x-coordinate of the second endpoint of the first line segment.
    * @param lineSegmentEnd1y   y-coordinate of the second endpoint of the first line segment.
    * @param lineSegmentStart2x x-coordinate of the first endpoint of the second line segment.
    * @param lineSegmentStart2y y-coordinate of the first endpoint of the second line segment.
    * @param lineSegmentEnd2x   x-coordinate of the second endpoint of the second line segment.
    * @param lineSegmentEnd2y   y-coordinate of the second endpoint of the second line segment.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    */
   public static boolean doLineSegment2DsIntersect(double lineSegmentStart1x,
                                                   double lineSegmentStart1y,
                                                   double lineSegmentEnd1x,
                                                   double lineSegmentEnd1y,
                                                   double lineSegmentStart2x,
                                                   double lineSegmentStart2y,
                                                   double lineSegmentEnd2x,
                                                   double lineSegmentEnd2y)
   {
      return intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1x,
                                                  lineSegmentStart1y,
                                                  lineSegmentEnd1x,
                                                  lineSegmentEnd1y,
                                                  lineSegmentStart2x,
                                                  lineSegmentStart2y,
                                                  lineSegmentEnd2x,
                                                  lineSegmentEnd2y,
                                                  null);
   }

   /**
    * Test if two line segments intersect each other.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, this method returns false.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart1 first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   second endpoint of the second line segment. Not modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    */
   public static boolean doLineSegment2DsIntersect(Point2DReadOnly lineSegmentStart1,
                                                   Point2DReadOnly lineSegmentEnd1,
                                                   Point2DReadOnly lineSegmentStart2,
                                                   Point2DReadOnly lineSegmentEnd2)
   {
      return doLineSegment2DsIntersect(lineSegmentStart1.getX(),
                                       lineSegmentStart1.getY(),
                                       lineSegmentEnd1.getX(),
                                       lineSegmentEnd1.getY(),
                                       lineSegmentStart2.getX(),
                                       lineSegmentStart2.getY(),
                                       lineSegmentEnd2.getX(),
                                       lineSegmentEnd2.getY());
   }

   /**
    * Tests if an intersection exists between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param rayOriginX        the x-coordinate of a point located on the ray. Not modified.
    * @param rayOriginY        the y-coordinate of a point located on the ray. Not modified.
    * @param rayDirectionX     the x-component of the direction of the ray. Not modified.
    * @param rayDirectionY     the y-component of the direction of the ray. Not modified.
    * @param lineSegmentStartX the x-coordinate of the first endpoint of the line segment.
    * @param lineSegmentStartY the y-coordinate of the first endpoint of the line segment.
    * @param lineSegmentEndX   the x-coordinate of the second endpoint of the line segment.
    * @param lineSegmentEndY   the y-coordinate of the second endpoint of the line segment.
    * @return {@code true} if the ray and line segment intersect, {@code false} otherwise.
    */
   public static boolean doRay2DAndLineSegment2DIntersect(double rayOriginX,
                                                          double rayOriginY,
                                                          double rayDirectionX,
                                                          double rayDirectionY,
                                                          double lineSegmentStartX,
                                                          double lineSegmentStartY,
                                                          double lineSegmentEndX,
                                                          double lineSegmentEndY)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOriginX,
                                                      rayOriginY,
                                                      rayDirectionX,
                                                      rayDirectionY,
                                                      lineSegmentStartX,
                                                      lineSegmentStartY,
                                                      lineSegmentEndX,
                                                      lineSegmentEndY,
                                                      null);
   }

   /**
    * Tests if an intersection exists between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param rayOrigin        a point located on the ray. Not modified.
    * @param rayDirection     the direction of the ray. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if the ray and line segment intersect, {@code false} otherwise.
    */
   public static boolean doRay2DAndLineSegment2DIntersect(Point2DReadOnly rayOrigin,
                                                          Vector2DReadOnly rayDirection,
                                                          Point2DReadOnly lineSegmentStart,
                                                          Point2DReadOnly lineSegmentEnd)
   {
      return doRay2DAndLineSegment2DIntersect(rayOrigin.getX(),
                                              rayOrigin.getY(),
                                              rayDirection.getX(),
                                              rayDirection.getY(),
                                              lineSegmentStart.getX(),
                                              lineSegmentStart.getY(),
                                              lineSegmentEnd.getX(),
                                              lineSegmentEnd.getY());
   }

   /**
    * Computes the dot product between two vectors each defined by two points:
    * <ul>
    * <li>{@code vector1 = end1 - start1}
    * <li>{@code vector2 = end2 - start2}
    * </ul>
    *
    * @param start1 the origin of the first vector. Not modified.
    * @param end1   the end of the first vector. Not modified.
    * @param start2 the origin of the second vector. Not modified.
    * @param end2   the end of the second vector. Not modified.
    * @return the value of the dot product of the two vectors.
    */
   public static double dotProduct(Point2DReadOnly start1, Point2DReadOnly end1, Point2DReadOnly start2, Point2DReadOnly end2)
   {
      double vector1X = end1.getX() - start1.getX();
      double vector1Y = end1.getY() - start1.getY();
      double vector2X = end2.getX() - start2.getX();
      double vector2Y = end2.getY() - start2.getY();

      return vector1X * vector2X + vector1Y * vector2Y;
   }

   /**
    * Computes the dot product between two vectors each defined by two points:
    * <ul>
    * <li>{@code vector1 = end1 - start1}
    * <li>{@code vector2 = end2 - start2}
    * </ul>
    *
    * @param start1 the origin of the first vector. Not modified.
    * @param end1   the end of the first vector. Not modified.
    * @param start2 the origin of the second vector. Not modified.
    * @param end2   the end of the second vector. Not modified.
    * @return the value of the dot product of the two vectors.
    */
   public static double dotProduct(Point3DReadOnly start1, Point3DReadOnly end1, Point3DReadOnly start2, Point3DReadOnly end2)
   {
      double vector1X = end1.getX() - start1.getX();
      double vector1Y = end1.getY() - start1.getY();
      double vector1Z = end1.getZ() - start1.getZ();
      double vector2X = end2.getX() - start2.getX();
      double vector2Y = end2.getY() - start2.getY();
      double vector2Z = end2.getZ() - start2.getZ();

      return vector1X * vector2X + vector1Y * vector2Y + vector1Z * vector2Z;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the bounding box are not restricted to exist between the two
    * given points defining the line.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is colinear with the bounding box boundary, the points where the line first/last
    * intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0 or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLine2DAndBoundingBox2D(Point2DReadOnly boundingBoxMin,
                                                               Point2DReadOnly boundingBoxMax,
                                                               Point2DReadOnly firstPointOnLine,
                                                               Point2DReadOnly secondPointOnLine,
                                                               Point2DBasics firstIntersectionToPack,
                                                               Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine2DAndBoundingBox2DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           firstPointOnLine.getX(),
                                                           firstPointOnLine.getY(),
                                                           true,
                                                           secondPointOnLine.getX(),
                                                           secondPointOnLine.getY(),
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is colinear with the bounding box boundary, the points where the line first/last
    * intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLine2DAndBoundingBox2D(Point2DReadOnly boundingBoxMin,
                                                               Point2DReadOnly boundingBoxMax,
                                                               Point2DReadOnly pointOnLine,
                                                               Vector2DReadOnly lineDirection,
                                                               Point2DBasics firstIntersectionToPack,
                                                               Point2DBasics secondIntersectionToPack)
   {
      double firstPointOnLineX = pointOnLine.getX();
      double firstPointOnLineY = pointOnLine.getY();
      double secondPointOnLineX = pointOnLine.getX() + lineDirection.getX();
      double secondPointOnLineY = pointOnLine.getY() + lineDirection.getY();
      return intersectionBetweenLine2DAndBoundingBox2DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           firstPointOnLineX,
                                                           firstPointOnLineY,
                                                           true,
                                                           secondPointOnLineX,
                                                           secondPointOnLineY,
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Flexible implementation for computing the intersection between a bounding box and either a line,
    * a line segment, or a ray.
    * <p>
    * Switching between line/line-segment/ray can be done using the two arguments
    * {@code canIntersectionOccurBeforeStart} and {@code canIntersectionOccurAfterEnd}:
    * <ul>
    * <li>{@code canIntersectionOccurBeforeStart == true} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate line/bounding-box
    * intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == false} changes the algorithm to calculate
    * line-segment/bounding-box intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate ray/bounding-box
    * intersection.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the ray origin or start/end point of a line segment lie on the boundary of the bounding
    * box they are considered as intersection points.
    * <li>if a line is colinear with a boundary of the bounding box, the points where the line
    * first/last intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    *
    * @param boundingBoxMin                  the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax                  the maximum coordinate of the bounding box. Not modified.
    * @param startX                          the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startY                          the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart specifies whether an intersection can exist before
    *                                        {@code start}.
    * @param endX                            the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endY                            the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurAfterEnd    specifies whether an intersection can exist after
    *                                        {@code end}.
    * @param firstIntersectionToPack         the coordinate of the first intersection. Can be
    *                                        {@code null}. Modified.
    * @param secondIntersectionToPack        the coordinate of the second intersection. Can be
    *                                        {@code null}. Modified.
    * @return the number of intersections between the line/line-segment/ray and the bounding box. It is
    *         either equal to 0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   private static int intersectionBetweenLine2DAndBoundingBox2DImpl(Point2DReadOnly boundingBoxMin,
                                                                    Point2DReadOnly boundingBoxMax,
                                                                    double startX,
                                                                    double startY,
                                                                    boolean canIntersectionOccurBeforeStart,
                                                                    double endX,
                                                                    double endY,
                                                                    boolean canIntersectionOccurAfterEnd,
                                                                    Point2DBasics firstIntersectionToPack,
                                                                    Point2DBasics secondIntersectionToPack)
   {
      if (boundingBoxMin.getX() > boundingBoxMax.getX() || boundingBoxMin.getY() > boundingBoxMax.getY())
         throw new BoundingBoxException(boundingBoxMin, boundingBoxMax);

      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      double dx = endX - startX;
      double dy = endY - startY;

      boolean isColinearX = false;
      boolean isColinearY = false;

      if (Math.abs(dx) < ONE_TRILLIONTH)
      {
         dx = 0.0 * dx;
         isColinearX = true;
      }
      if (Math.abs(dy) < ONE_TRILLIONTH)
      {
         dy = 0.0 * dy;
         isColinearY = true;
      }

      double invXDir = 1.0 / dx;
      double invYDir = 1.0 / dy;

      double tmin, tmax, tymin, tymax;

      double deltaXmin = boundingBoxMin.getX() - startX;
      double deltaXmax = boundingBoxMax.getX() - startX;

      if (Math.abs(deltaXmin) < ONE_TRILLIONTH)
      {
         deltaXmin = 0.0 * deltaXmin;
      }
      if (Math.abs(deltaXmax) < ONE_TRILLIONTH)
      {
         deltaXmax = 0.0 * deltaXmax;
      }

      if (invXDir > 0.0)
      {
         tmin = deltaXmin * invXDir;
         tmax = deltaXmax * invXDir;
      }
      else
      {
         tmin = deltaXmax * invXDir;
         tmax = deltaXmin * invXDir;
      }

      double deltaYmin = boundingBoxMin.getY() - startY;
      double deltaYmax = boundingBoxMax.getY() - startY;

      if (Math.abs(deltaYmin) < ONE_TRILLIONTH)
      {
         deltaYmin = 0.0 * deltaYmin;
      }
      if (Math.abs(deltaYmax) < ONE_TRILLIONTH)
      {
         deltaYmax = 0.0 * deltaYmax;
      }

      if (invYDir > 0.0)
      {
         tymin = deltaYmin * invYDir;
         tymax = deltaYmax * invYDir;
      }
      else
      {
         tymin = deltaYmax * invYDir;
         tymax = deltaYmin * invYDir;
      }

      // if regions do not overlap, return false
      if (tmin > tymax + ONE_TRILLIONTH)
      {
         return 0;
      }
      if (tmax < tymin - ONE_TRILLIONTH)
      {
         return 0;
      }

      // update tmin - make sure its not NaN
      if (tymin > tmin || Double.isNaN(tmin))
         tmin = tymin;

      if (tymax < tmax || Double.isNaN(tmax))
         tmax = tymax;

      // From here, we know that the support line is intersecting with the bounding-box.
      int numberOfIntersections = 0;

      if (!canIntersectionOccurAfterEnd && tmin > 1.0)
         return 0;
      if (!canIntersectionOccurBeforeStart && tmax < 0.0)
         return 0;

      boolean isIntersectingAtTmin = canIntersectionOccurBeforeStart || tmin >= 0.0;
      boolean isIntersectingAtTmax = canIntersectionOccurAfterEnd || tmax <= 1.0;

      if (isIntersectingAtTmin)
         numberOfIntersections++;
      if (isIntersectingAtTmax)
         numberOfIntersections++;

      // 1 intersection
      if (numberOfIntersections == 1)
      {
         if (firstIntersectionToPack != null)
         {
            if (isIntersectingAtTmin)
            {
               firstIntersectionToPack.set(tmin * dx + startX, tmin * dy + startY);
            }
            else
            {
               firstIntersectionToPack.set(tmax * dx + startX, tmax * dy + startY);
            }
         }

         if (secondIntersectionToPack != null)
            secondIntersectionToPack.setToNaN();
      }

      // 2 intersections
      if (numberOfIntersections == 2)
      {
         if (firstIntersectionToPack != null)
         {
            firstIntersectionToPack.set(tmin * dx + startX, tmin * dy + startY);
         }

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.set(tmax * dx + startX, tmax * dy + startY);
         }
      }

      // check if the two intersection points are (almost) identical
      if (numberOfIntersections == 2 && EuclidCoreTools.epsilonEquals(tmin, tmax, ONE_TRILLIONTH))
      {
         numberOfIntersections = 1;

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.setToNaN();
         }
         return numberOfIntersections;
      }

      if (canIntersectionOccurAfterEnd && canIntersectionOccurBeforeStart)
      {// we have a infinite line and we are done here
         return numberOfIntersections;
      }

      if (!isColinearX && !isColinearY)
      { // No edge case, we good
         return numberOfIntersections;
      }

      // if we get here we have a line-segment or a ray colinear with one of the box surfaces and we need to check for edge cases
      if (canIntersectionOccurAfterEnd)
      {// we have a ray and need to test for the edge-case that can only occur with ray at this point
       // check if ray origin lies on bounding box surface and ray or line are colinear  with that bounding box surface
         boolean isRayOriginOnBox = false;

         if (isColinearX)
         {// check if start point of ray lies on bounding box surface
            if (deltaXmin == 0.0 || deltaXmax == 0.0)
            {
               if (deltaYmin <= 0.0 && deltaYmax >= 0.0)
               { // ray origin lies on x surface of box
                  isRayOriginOnBox = true;
               }
            }
         }
         else if (isColinearY)
         {// check if start point of ray lies on bounding box surface
            if (deltaYmin == 0.0 || deltaYmax == 0.0)
            {
               if (deltaXmin <= 0.0 && deltaXmax >= 0.0)
               { // ray origin lies on y surface of box
                  isRayOriginOnBox = true;
               }
            }
         }

         if (isRayOriginOnBox)
         {// the ray origin lies on the bounding box surface, we consider the ray origin the first intersection
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(firstIntersectionToPack);
            }
            if (firstIntersectionToPack != null)
            {
               firstIntersectionToPack.set(startX, startY);
               tmin = 0.0;
            }
            numberOfIntersections = 2;
         }
      }
      else
      {// we have a line-segment and need to test for the edge-case that can only occur with a line-segment at this point
       // check if start- and/or end- point lies on bounding box surface and the line-segment is colinear with that bounding box surface
         boolean isLineSegmentStartOnBox = false;
         boolean isLineSegmentEndOnBox = false;
         double deltaXEndmin = boundingBoxMin.getX() - endX;
         double deltaXEndmax = boundingBoxMax.getX() - endX;
         double deltaYEndmin = boundingBoxMin.getY() - endY;
         double deltaYEndmax = boundingBoxMax.getY() - endY;

         if (isColinearX)
         {// check if start point of line-segment lies on bounding box surface   

            if (deltaYmin <= 0.0 && deltaYmax >= 0.0)
            { // line-segment start point lies on x surface of box
               isLineSegmentStartOnBox = true;
            }
            if (deltaYEndmin <= 0.0 && deltaYEndmax >= 0.0)
            { // line-segment end point also lies on x surface of box
               isLineSegmentEndOnBox = true;
            }

         }
         else if (isColinearY)
         {// check if start point of ray lies on bounding box surface

            if (deltaXmin <= 0.0 && deltaXmax >= 0.0)
            { // line-segment start point lies on x surface of box
               isLineSegmentStartOnBox = true;
            }
            if (deltaXEndmin <= 0.0 && deltaXEndmax >= 0.0)
            { // line-segment end point also lies on x surface of box
               isLineSegmentEndOnBox = true;

            }
         }

         if (isLineSegmentStartOnBox && isLineSegmentEndOnBox)
         { // start and end are on the box, we consider them intersections
            if (firstIntersectionToPack != null)
            {
               firstIntersectionToPack.set(startX, startY);
               tmin = 0.0;
            }
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(endX, endY);
               tmax = 1.0;
            }
            numberOfIntersections = 2;
         }
         else if (isLineSegmentStartOnBox)
         { // only the  start is on the box, we consider the segment start as the first intersection
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(firstIntersectionToPack);
            }
            if (firstIntersectionToPack != null)
            {
               firstIntersectionToPack.set(startX, startY);
               tmin = 0.0;
            }
            numberOfIntersections = 2;
         }
         else
         { // only the  end is on the box, we consider the segment end as the first intersection
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(endX, endY);
               tmax = 1.0;
            }
            numberOfIntersections = 2;
         }
      }

      // check if the two intersection points are (almost) identical
      if (numberOfIntersections == 2 && EuclidCoreTools.epsilonEquals(tmin, tmax, ONE_TRILLIONTH))
      {
         numberOfIntersections = 1;

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.setToNaN();
         }
      }
      return numberOfIntersections;
   }

   /**
    * Computes the intersection between an infinitely long 2D line (defined by a 2D point and a 2D
    * direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLineX       x-coordinate of a point located on the line.
    * @param pointOnLineY       y-coordinate of a point located on the line.
    * @param lineDirectionX     x-component of the line direction.
    * @param lineDirectionY     y-component of the line direction.
    * @param lineSegmentStartX  x-coordinate of the first endpoint of the line segment.
    * @param lineSegmentStartY  y-coordinate of the first endpoint of the line segment.
    * @param lineSegmentEndX    x-coordinate of the second endpoint of the line segment.
    * @param lineSegmentEndY    y-coordinate of the second endpoint of the line segment.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    */
   public static boolean intersectionBetweenLine2DAndLineSegment2D(double pointOnLineX,
                                                                   double pointOnLineY,
                                                                   double lineDirectionX,
                                                                   double lineDirectionY,
                                                                   double lineSegmentStartX,
                                                                   double lineSegmentStartY,
                                                                   double lineSegmentEndX,
                                                                   double lineSegmentEndY,
                                                                   Point2DBasics intersectionToPack)
   {
      double lineSegmentDirectionX = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDirectionY = lineSegmentEndY - lineSegmentStartY;

      double percentage = percentageOfIntersectionBetweenTwoLine2Ds(lineSegmentStartX,
                                                                    lineSegmentStartY,
                                                                    lineSegmentDirectionX,
                                                                    lineSegmentDirectionY,
                                                                    pointOnLineX,
                                                                    pointOnLineY,
                                                                    lineDirectionX,
                                                                    lineDirectionY);
      if (Double.isInfinite(percentage))
      { // The line and the segment are collinear, let's set the intersection to the start of the segment.
         if (intersectionToPack != null)
            intersectionToPack.set(lineSegmentStartX, lineSegmentStartY);
         return true;
      }

      if (Double.isNaN(percentage) || percentage < 0.0 - ONE_TEN_MILLIONTH || percentage > 1.0 + ONE_TEN_MILLIONTH)
      {
         if (intersectionToPack != null)
            intersectionToPack.setToNaN();
         return false;
      }

      if (intersectionToPack != null)
      {
         intersectionToPack.set(EuclidCoreTools.interpolate(lineSegmentStartX, lineSegmentEndX, percentage),
                                EuclidCoreTools.interpolate(lineSegmentStartY, lineSegmentEndY, percentage));
      }
      return true;
   }

   /**
    * Flexible implementation for computing the intersection between a ray/line/line-segment and
    * ray/line/line-segment.
    * <p>
    * Switching between line/line-segment/ray can be done using the arguments
    * {@code canIntersectionOccurBeforeStart1}, {@code canIntersectionOccurBeforeEnd1},
    * {@code canIntersectionOccurBeforeStart2}, and {@code canIntersectionOccurBeforeEnd2}.
    * <ul>
    *
    * @param start1x                          the x-coordinate of a point located on the first
    *                                         line/line-segment/ray.
    * @param start1y                          the y-coordinate of a point located on the first
    *                                         line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart1 specifies whether an intersection can exist before
    *                                         {@code start}.
    * @param end1x                            the x-coordinate of a point located on the first
    *                                         line/line-segment/ray.
    * @param end1y                            the y-coordinate of a point located on the first
    *                                         line/line-segment/ray.
    * @param canIntersectionOccurBeforeEnd1   specifies whether an intersection can exist after
    *                                         {@code end}.
    * @param start2x                          the x-coordinate of a point located on the second
    *                                         line/line-segment/ray.
    * @param start2y                          the y-coordinate of a point located on the second
    *                                         line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart2 specifies whether an intersection can exist before
    *                                         {@code start}.
    * @param end2x                            the x-coordinate of a point located on the second
    *                                         line/line-segment/ray.
    * @param end2y                            the y-coordinate of a point located on the second
    *                                         line/line-segment/ray.
    * @param canIntersectionOccurBeforeEnd2   specifies whether an intersection can exist after
    *                                         {@code end}.
    * @param intersectionToPack               the coordinate of the intersection. Can be {@code null}.
    *                                         Modified.
    * @return whether there is an intersection or not.
    */
   private static boolean intersectionBetweenTwoLine2DsImpl(double start1x,
                                                            double start1y,
                                                            boolean canIntersectionOccurBeforeStart1,
                                                            double end1x,
                                                            double end1y,
                                                            boolean canIntersectionOccurBeforeEnd1,
                                                            double start2x,
                                                            double start2y,
                                                            boolean canIntersectionOccurBeforeStart2,
                                                            double end2x,
                                                            double end2y,
                                                            boolean canIntersectionOccurBeforeEnd2,
                                                            Point2DBasics intersectionToPack)
   {
      double epsilon = ONE_TEN_MILLIONTH;

      double direction1x = end1x - start1x;
      double direction1y = end1y - start1y;
      double direction2x = end2x - start2x;
      double direction2y = end2y - start2y;

      double determinant = -direction1x * direction2y + direction1y * direction2x;

      double zeroish = 0.0 - epsilon;

      if (Math.abs(determinant) < epsilon)
      { // The lines are parallel
        // Check if they are collinear
         double dx = start2x - start1x;
         double dy = start2y - start1y;
         double cross = dx * direction1y - dy * direction1x;

         if (Math.abs(cross) < epsilon)
         {
            if (canIntersectionOccurBeforeStart1 && canIntersectionOccurBeforeEnd1)
            { // (start1, end1) represents a line
               if (canIntersectionOccurBeforeStart2 && canIntersectionOccurBeforeEnd2)
               { // (start2, end2) represents a line
                  if (intersectionToPack != null)
                     intersectionToPack.set(start1x, start1y);
                  return true;
               }

               if (intersectionToPack != null)
                  intersectionToPack.set(start2x, start2y);
               return true;
            }

            if (canIntersectionOccurBeforeStart2 && canIntersectionOccurBeforeEnd2)
            { // (start2, end2) represents a line
               if (intersectionToPack != null)
                  intersectionToPack.set(start1x, start1y);
               return true;
            }

            // Let's find the first endpoint that is inside the other line segment and return it.
            double direction1LengthSquare = EuclidCoreTools.normSquared(direction1x, direction1y);
            double dot;

            // Check if start2 is inside (start1, end1)
            dx = start2x - start1x;
            dy = start2y - start1y;
            dot = dx * direction1x + dy * direction1y;

            if ((canIntersectionOccurBeforeStart1 || zeroish < dot) && (canIntersectionOccurBeforeEnd1 || dot < direction1LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(start2x, start2y);
               return true;
            }

            // Check if end2 is inside (start1, end1)
            dx = end2x - start1x;
            dy = end2y - start1y;
            dot = dx * direction1x + dy * direction1y;

            if ((canIntersectionOccurBeforeStart1 || zeroish < dot) && (canIntersectionOccurBeforeEnd1 || dot < direction1LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(end2x, end2y);
               return true;
            }

            double direction2LengthSquare = EuclidCoreTools.normSquared(direction2x, direction2y);

            // Check if start1 is inside (start2, end2)
            dx = start1x - start2x;
            dy = start1y - start2y;
            dot = dx * direction2x + dy * direction2y;

            if ((canIntersectionOccurBeforeStart2 || zeroish < dot) && (canIntersectionOccurBeforeEnd2 || dot < direction2LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(start1x, start1y);
               return true;
            }

            // Check if end1 is inside (start2, end2)
            dx = end1x - start2x;
            dy = end1y - start2y;
            dot = dx * direction2x + dy * direction2y;

            if ((canIntersectionOccurBeforeStart2 || zeroish < dot) && (canIntersectionOccurBeforeEnd2 || dot < direction2LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(end1x, end1y);
               return true;
            }

            // (start1, end1) and (start2, end2) represent ray and/or line segment and they are collinear but do not overlap.
            if (intersectionToPack != null)
               intersectionToPack.setToNaN();
            return false;
         }
         // The lines are parallel but are not collinear, they do not intersect.
         else
         {
            if (intersectionToPack != null)
               intersectionToPack.setToNaN();
            return false;
         }
      }

      double dx = start2x - start1x;
      double dy = start2y - start1y;

      double oneOverDeterminant = 1.0 / determinant;
      double AInverse00 = -direction2y;
      double AInverse01 = direction2x;
      double AInverse10 = -direction1y;
      double AInverse11 = direction1x;

      double alpha = oneOverDeterminant * (AInverse00 * dx + AInverse01 * dy);
      double beta = oneOverDeterminant * (AInverse10 * dx + AInverse11 * dy);

      double oneish = 1.0 + epsilon;

      boolean areIntersecting = (canIntersectionOccurBeforeStart1 || zeroish < alpha) && (canIntersectionOccurBeforeEnd1 || alpha < oneish);
      if (areIntersecting)
         areIntersecting = (canIntersectionOccurBeforeStart2 || zeroish < beta) && (canIntersectionOccurBeforeEnd2 || beta < oneish);

      if (intersectionToPack != null)
      {
         if (areIntersecting)
         {
            intersectionToPack.set(start1x + alpha * direction1x, start1y + alpha * direction1y);
         }
         else
         {
            intersectionToPack.setToNaN();
         }
      }

      return areIntersecting;
   }

   /**
    * Computes the intersection between an infinitely long 2D line (defined by a 2D point and a 2D
    * direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect,
    * this method returns {@code null}.
    * <li>When the line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns that
    * same endpoint.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the line direction. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return the 2D point of intersection if it exist, {@code null} otherwise.
    */
   public static Point2D intersectionBetweenLine2DAndLineSegment2D(Point2DReadOnly pointOnLine,
                                                                   Vector2DReadOnly lineDirection,
                                                                   Point2DReadOnly lineSegmentStart,
                                                                   Point2DReadOnly lineSegmentEnd)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, intersection);
      if (!success)
         return null;
      else
         return intersection;
   }

   /**
    * Computes the intersection between an infinitely long 2D line (defined by a 2D point and a 2D
    * direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the line direction. Not modified.
    * @param lineSegmentStart   the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd     the second endpoint of the line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    */
   public static boolean intersectionBetweenLine2DAndLineSegment2D(Point2DReadOnly pointOnLine,
                                                                   Vector2DReadOnly lineDirection,
                                                                   Point2DReadOnly lineSegmentStart,
                                                                   Point2DReadOnly lineSegmentEnd,
                                                                   Point2DBasics intersectionToPack)
   {
      return intersectionBetweenLine2DAndLineSegment2D(pointOnLine.getX(),
                                                       pointOnLine.getY(),
                                                       lineDirection.getX(),
                                                       lineDirection.getY(),
                                                       lineSegmentStart.getX(),
                                                       lineSegmentStart.getY(),
                                                       lineSegmentEnd.getX(),
                                                       lineSegmentEnd.getY(),
                                                       intersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the bounding box are not restricted to exist between the two
    * given points defining the line.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is on a face of the bounding box, the points where the line first/last intersects
    * with the bounding box are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(Point3DReadOnly boundingBoxMin,
                                                               Point3DReadOnly boundingBoxMax,
                                                               Point3DReadOnly firstPointOnLine,
                                                               Point3DReadOnly secondPointOnLine,
                                                               Point3DBasics firstIntersectionToPack,
                                                               Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           firstPointOnLine.getX(),
                                                           firstPointOnLine.getY(),
                                                           firstPointOnLine.getZ(),
                                                           true,
                                                           secondPointOnLine.getX(),
                                                           secondPointOnLine.getY(),
                                                           secondPointOnLine.getZ(),
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is on a face of the bounding box, the points where the line first/last intersects
    * with the bounding box are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(Point3DReadOnly boundingBoxMin,
                                                               Point3DReadOnly boundingBoxMax,
                                                               Point3DReadOnly pointOnLine,
                                                               Vector3DReadOnly lineDirection,
                                                               Point3DBasics firstIntersectionToPack,
                                                               Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin.getX(),
                                                       boundingBoxMin.getY(),
                                                       boundingBoxMin.getZ(),
                                                       boundingBoxMax.getX(),
                                                       boundingBoxMax.getY(),
                                                       boundingBoxMax.getZ(),
                                                       pointOnLine,
                                                       lineDirection,
                                                       firstIntersectionToPack,
                                                       secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is on a face of the bounding box, the points where the line first/last intersects
    * with the bounding box are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMinX          the minimum x-coordinate of the bounding box.
    * @param boundingBoxMinY          the minimum y-coordinate of the bounding box.
    * @param boundingBoxMinZ          the minimum z-coordinate of the bounding box.
    * @param boundingBoxMaxX          the maximum x-coordinate of the bounding box.
    * @param boundingBoxMaxY          the maximum y-coordinate of the bounding box.
    * @param boundingBoxMaxZ          the maximum z-coordinate of the bounding box.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(double boundingBoxMinX,
                                                               double boundingBoxMinY,
                                                               double boundingBoxMinZ,
                                                               double boundingBoxMaxX,
                                                               double boundingBoxMaxY,
                                                               double boundingBoxMaxZ,
                                                               Point3DReadOnly pointOnLine,
                                                               Vector3DReadOnly lineDirection,
                                                               Point3DBasics firstIntersectionToPack,
                                                               Point3DBasics secondIntersectionToPack)
   {
      double firstPointOnLineX = pointOnLine.getX();
      double firstPointOnLineY = pointOnLine.getY();
      double firstPointOnLineZ = pointOnLine.getZ();
      double secondPointOnLineX = pointOnLine.getX() + lineDirection.getX();
      double secondPointOnLineY = pointOnLine.getY() + lineDirection.getY();
      double secondPointOnLineZ = pointOnLine.getZ() + lineDirection.getZ();
      return intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMinX,
                                                           boundingBoxMinY,
                                                           boundingBoxMinZ,
                                                           boundingBoxMaxX,
                                                           boundingBoxMaxY,
                                                           boundingBoxMaxZ,
                                                           firstPointOnLineX,
                                                           firstPointOnLineY,
                                                           firstPointOnLineZ,
                                                           true,
                                                           secondPointOnLineX,
                                                           secondPointOnLineY,
                                                           secondPointOnLineZ,
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is on a face of the bounding box, the points where the line first/last intersects
    * with the bounding box are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMinX          the minimum x-coordinate of the bounding box.
    * @param boundingBoxMinY          the minimum y-coordinate of the bounding box.
    * @param boundingBoxMinZ          the minimum z-coordinate of the bounding box.
    * @param boundingBoxMaxX          the maximum x-coordinate of the bounding box.
    * @param boundingBoxMaxY          the maximum y-coordinate of the bounding box.
    * @param boundingBoxMaxZ          the maximum z-coordinate of the bounding box.
    * @param pointOnLineX             the x-coordinate of a point located on the infinitely long line.
    * @param pointOnLineY             the y-coordinate of a point located on the infinitely long line.
    * @param pointOnLineZ             the z-coordinate of a point located on the infinitely long line.
    * @param lineDirectionX           the x-component of the direction of the line.
    * @param lineDirectionY           the y-component of the direction of the line.
    * @param lineDirectionZ           the z-component of the direction of the line.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(double boundingBoxMinX,
                                                               double boundingBoxMinY,
                                                               double boundingBoxMinZ,
                                                               double boundingBoxMaxX,
                                                               double boundingBoxMaxY,
                                                               double boundingBoxMaxZ,
                                                               double pointOnLineX,
                                                               double pointOnLineY,
                                                               double pointOnLineZ,
                                                               double lineDirectionX,
                                                               double lineDirectionY,
                                                               double lineDirectionZ,
                                                               Point3DBasics firstIntersectionToPack,
                                                               Point3DBasics secondIntersectionToPack)
   {
      double firstPointOnLineX = pointOnLineX;
      double firstPointOnLineY = pointOnLineY;
      double firstPointOnLineZ = pointOnLineZ;
      double secondPointOnLineX = pointOnLineX + lineDirectionX;
      double secondPointOnLineY = pointOnLineY + lineDirectionY;
      double secondPointOnLineZ = pointOnLineZ + lineDirectionZ;
      return intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMinX,
                                                           boundingBoxMinY,
                                                           boundingBoxMinZ,
                                                           boundingBoxMaxX,
                                                           boundingBoxMaxY,
                                                           boundingBoxMaxZ,
                                                           firstPointOnLineX,
                                                           firstPointOnLineY,
                                                           firstPointOnLineZ,
                                                           true,
                                                           secondPointOnLineX,
                                                           secondPointOnLineY,
                                                           secondPointOnLineZ,
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Flexible implementation for computing the intersection between a bounding box and either a line,
    * a line segment, or a ray.
    * <p>
    * Switching between line/line-segment/ray can be done using the two arguments
    * {@code canIntersectionOccurBeforeStart} and {@code canIntersectionOccurAfterEnd}:
    * <ul>
    * <li>{@code canIntersectionOccurBeforeStart == true} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate line/bounding-box
    * intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == false} changes the algorithm to calculate
    * line-segment/bounding-box intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate ray/bounding-box
    * intersection.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the ray origin or start/end point of a line segment lie on the surface of the bounding box
    * they are considered as intersection points.
    * <li>if a line is on a face of the bounding box, the points where the line first/last intersects
    * with the bounding box (on the bounding box boundary) are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMin                  the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax                  the maximum coordinate of the bounding box. Not modified.
    * @param startX                          the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startY                          the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startZ                          the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart specifies whether an intersection can exist before
    *                                        {@code start}.
    * @param endX                            the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endY                            the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endZ                            the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurAfterEnd    specifies whether an intersection can exist after
    *                                        {@code end}.
    * @param firstIntersectionToPack         the coordinate of the first intersection. Can be
    *                                        {@code null}. Modified.
    * @param secondIntersectionToPack        the coordinate of the second intersection. Can be
    *                                        {@code null}. Modified.
    * @return the number of intersections between the line/line-segment/ray and the bounding box. It is
    *         either equal to 0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   private static int intersectionBetweenLine3DAndBoundingBox3DImpl(Point3DReadOnly boundingBoxMin,
                                                                    Point3DReadOnly boundingBoxMax,
                                                                    double startX,
                                                                    double startY,
                                                                    double startZ,
                                                                    boolean canIntersectionOccurBeforeStart,
                                                                    double endX,
                                                                    double endY,
                                                                    double endZ,
                                                                    boolean canIntersectionOccurAfterEnd,
                                                                    Point3DBasics firstIntersectionToPack,
                                                                    Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMin.getX(),
                                                           boundingBoxMin.getY(),
                                                           boundingBoxMin.getZ(),
                                                           boundingBoxMax.getX(),
                                                           boundingBoxMax.getY(),
                                                           boundingBoxMax.getZ(),
                                                           startX,
                                                           startY,
                                                           startZ,
                                                           canIntersectionOccurBeforeStart,
                                                           endX,
                                                           endY,
                                                           endZ,
                                                           canIntersectionOccurAfterEnd,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Flexible implementation for computing the intersection between a bounding box and either a line,
    * a line segment, or a ray.
    * <p>
    * Switching between line/line-segment/ray can be done using the two arguments
    * {@code canIntersectionOccurBeforeStart} and {@code canIntersectionOccurAfterEnd}:
    * <ul>
    * <li>{@code canIntersectionOccurBeforeStart == true} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate line/bounding-box
    * intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == false} changes the algorithm to calculate
    * line-segment/bounding-box intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate ray/bounding-box
    * intersection.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the ray origin or start/end point of a line segment lie on the surface of the bounding box
    * they are considered as intersection points.
    * <li>if the line is on a face of the bounding box, the points where the line first/last intersects
    * with the bounding box (on the bounding box boundary) are returned as intersection points.
    * </ul>
    * </p>
    *
    * @param boundingBoxMinX                 the minimum x-coordinate of the bounding box.
    * @param boundingBoxMinY                 the minimum y-coordinate of the bounding box.
    * @param boundingBoxMinZ                 the minimum z-coordinate of the bounding box.
    * @param boundingBoxMaxX                 the maximum x-coordinate of the bounding box.
    * @param boundingBoxMaxY                 the maximum y-coordinate of the bounding box.
    * @param boundingBoxMaxZ                 the maximum z-coordinate of the bounding box.
    * @param startX                          the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startY                          the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startZ                          the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart specifies whether an intersection can exist before
    *                                        {@code start}.
    * @param endX                            the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endY                            the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endZ                            the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurAfterEnd    specifies whether an intersection can exist after
    *                                        {@code end}.
    * @param firstIntersectionToPack         the coordinate of the first intersection. Can be
    *                                        {@code null}. Modified.
    * @param secondIntersectionToPack        the coordinate of the second intersection. Can be
    *                                        {@code null}. Modified.
    * @return the number of intersections between the line/line-segment/ray and the bounding box. It is
    *         either equal to 0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   private static int intersectionBetweenLine3DAndBoundingBox3DImpl(double boundingBoxMinX,
                                                                    double boundingBoxMinY,
                                                                    double boundingBoxMinZ,
                                                                    double boundingBoxMaxX,
                                                                    double boundingBoxMaxY,
                                                                    double boundingBoxMaxZ,
                                                                    double startX,
                                                                    double startY,
                                                                    double startZ,
                                                                    boolean canIntersectionOccurBeforeStart,
                                                                    double endX,
                                                                    double endY,
                                                                    double endZ,
                                                                    boolean canIntersectionOccurAfterEnd,
                                                                    Point3DBasics firstIntersectionToPack,
                                                                    Point3DBasics secondIntersectionToPack)
   {
      if (boundingBoxMinX > boundingBoxMaxX || boundingBoxMinY > boundingBoxMaxY || boundingBoxMinZ > boundingBoxMaxZ)
         throw new BoundingBoxException(boundingBoxMinX, boundingBoxMinY, boundingBoxMinZ, boundingBoxMaxX, boundingBoxMaxY, boundingBoxMaxZ);

      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      double dx = endX - startX;
      double dy = endY - startY;
      double dz = endZ - startZ;

      boolean isColinearX = false;
      boolean isColinearY = false;
      boolean isColinearZ = false;

      if (Math.abs(dx) < ONE_TRILLIONTH)
      {
         dx = 0.0 * dx;
         isColinearX = true;
      }
      if (Math.abs(dy) < ONE_TRILLIONTH)
      {
         dy = 0.0 * dy;
         isColinearY = true;
      }

      if (Math.abs(dz) < ONE_TRILLIONTH)
      {
         dz = 0.0 * dz;
         isColinearZ = true;
      }

      double invXDir = 1.0 / dx;
      double invYDir = 1.0 / dy;
      double invZDir = 1.0 / dz;

      double tmin, tmax, tymin, tymax, tzmin, tzmax;

      double deltaXmin = boundingBoxMinX - startX;
      double deltaXmax = boundingBoxMaxX - startX;

      if (Math.abs(deltaXmin) < ONE_TRILLIONTH)
      {
         deltaXmin = 0.0 * deltaXmin;
      }
      if (Math.abs(deltaXmax) < ONE_TRILLIONTH)
      {
         deltaXmax = 0.0 * deltaXmax;
      }

      if (invXDir > 0.0)
      {
         tmin = deltaXmin * invXDir;
         tmax = deltaXmax * invXDir;
      }
      else
      {
         tmin = deltaXmax * invXDir;
         tmax = deltaXmin * invXDir;
      }

      double deltaYmin = boundingBoxMinY - startY;
      double deltaYmax = boundingBoxMaxY - startY;

      if (Math.abs(deltaYmin) < ONE_TRILLIONTH)
      {
         deltaYmin = 0.0 * deltaYmin;
      }
      if (Math.abs(deltaYmax) < ONE_TRILLIONTH)
      {
         deltaYmax = 0.0 * deltaYmax;
      }

      if (invYDir > 0.0)
      {
         tymin = deltaYmin * invYDir;
         tymax = deltaYmax * invYDir;
      }
      else
      {
         tymin = deltaYmax * invYDir;
         tymax = deltaYmin * invYDir;
      }

      // if regions do not overlap, return false
      if (tmin > tymax + ONE_TRILLIONTH)
      {
         return 0;
      }
      if (tmax < tymin - ONE_TRILLIONTH)
      {
         return 0;
      }

      // update tmin - make sure its not NaN
      if (tymin > tmin || Double.isNaN(tmin))
         tmin = tymin;

      if (tymax < tmax || Double.isNaN(tmax))
         tmax = tymax;

      double deltaZmin = boundingBoxMinZ - startZ;
      double deltaZmax = boundingBoxMaxZ - startZ;

      if (Math.abs(deltaZmin) < ONE_TRILLIONTH)
      {
         deltaZmin = 0.0 * deltaZmin;
      }
      if (Math.abs(deltaZmax) < ONE_TRILLIONTH)
      {
         deltaZmax = 0.0 * deltaZmax;
      }

      if (invZDir > 0.0)
      {
         tzmin = deltaZmin * invZDir;
         tzmax = deltaZmax * invZDir;
      }
      else
      {
         tzmin = deltaZmax * invZDir;
         tzmax = deltaZmin * invZDir;
      }

      // if regions do not overlap, return false
      if (tmin > tzmax + ONE_TRILLIONTH)
      {
         return 0;
      }
      if (tmax < tzmin - ONE_TRILLIONTH)
      {
         return 0;
      }

      // update tmin
      if (tzmin > tmin || Double.isNaN(tmin))
         tmin = tzmin;

      if (tzmax < tmax || Double.isNaN(tmax))
         tmax = tzmax;

      // From here, we know that the support line is intersecting with the bounding-box.
      int numberOfIntersections = 0;

      if (!canIntersectionOccurAfterEnd && tmin > 1.0)
         return 0;
      if (!canIntersectionOccurBeforeStart && tmax < 0.0)
         return 0;

      boolean isIntersectingAtTmin = canIntersectionOccurBeforeStart || tmin >= 0.0;
      boolean isIntersectingAtTmax = canIntersectionOccurAfterEnd || tmax <= 1.0;

      if (isIntersectingAtTmin)
         numberOfIntersections++;
      if (isIntersectingAtTmax)
         numberOfIntersections++;

      // 1 intersection
      if (numberOfIntersections == 1)
      {
         if (firstIntersectionToPack != null)
         {
            if (isIntersectingAtTmin)
            {
               firstIntersectionToPack.set(tmin * dx + startX, tmin * dy + startY, tmin * dz + startZ);
            }
            else
            {
               firstIntersectionToPack.set(tmax * dx + startX, tmax * dy + startY, tmax * dz + startZ);
            }
         }

         if (secondIntersectionToPack != null)
            secondIntersectionToPack.setToNaN();
      }

      // 2 intersections
      if (numberOfIntersections == 2)
      {
         if (firstIntersectionToPack != null)
         {
            firstIntersectionToPack.set(tmin * dx + startX, tmin * dy + startY, tmin * dz + startZ);
         }

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.set(tmax * dx + startX, tmax * dy + startY, tmax * dz + startZ);
         }
      }

      // check if the two intersection points are (almost) identical
      if (numberOfIntersections == 2 && EuclidCoreTools.epsilonEquals(tmin, tmax, ONE_TRILLIONTH))
      {
         numberOfIntersections = 1;

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.setToNaN();
         }
         return numberOfIntersections;
      }

      if (canIntersectionOccurAfterEnd && canIntersectionOccurBeforeStart)
      {// we have a infinite line and we are done here
         return numberOfIntersections;
      }

      if (!isColinearX && !isColinearY && !isColinearZ)
      { // No edge case, we good
         return numberOfIntersections;
      }

      // if we get here we have a line-segment or a ray colinear with one of the box surfaces and we need to check for edge cases
      if (canIntersectionOccurAfterEnd)
      {// we have a ray and need to test for the edge-case that can only occur with ray at this point
       // check if ray origin lies on bounding box surface and ray or line are colinear  with that bounding box surface
         boolean isRayOriginOnBox = false;

         if (isColinearX)
         {// check if start point of ray lies on bounding box surface
            if (deltaXmin == 0.0 || deltaXmax == 0.0)
            {
               if (deltaYmin <= 0.0 && deltaYmax >= 0.0 && deltaZmin <= 0.0 && deltaZmax >= 0.0)
               { // ray origin lies on x surface of box
                  isRayOriginOnBox = true;
               }
            }
         }
         else if (isColinearY)
         {// check if start point of ray lies on bounding box surface
            if (deltaYmin == 0.0 || deltaYmax == 0.0)
            {
               if (deltaXmin <= 0.0 && deltaXmax >= 0.0 && deltaZmin <= 0.0 && deltaZmax >= 0.0)
               { // ray origin lies on y surface of box
                  isRayOriginOnBox = true;
               }
            }
         }
         else if (isColinearZ)
         {// check if start point of ray lies on bounding box surface
            if (deltaZmin == 0.0 || deltaZmax == 0.0)
            {
               if (deltaYmin <= 0.0 && deltaYmax >= 0.0 && deltaXmin <= 0.0 && deltaXmax >= 0.0)
               { // ray origin lies on z surface of box
                  isRayOriginOnBox = true;
               }
            }
         }

         if (isRayOriginOnBox)
         {// the ray origin lies on the bounding box surface, we consider the ray origin the first intersection
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(firstIntersectionToPack);
            }
            if (firstIntersectionToPack != null)
            {
               firstIntersectionToPack.set(startX, startY, startZ);
               tmin = 0.0;
            }
            numberOfIntersections = 2;
         }
      }
      else
      {// we have a line-segment and need to test for the edge-case that can only occur with a line-segment at this point
       // check if start- and/or end- point lies on bounding box surface and the line-segment is colinear with that bounding box surface
         boolean isLineSegmentStartOnBox = false;
         boolean isLineSegmentEndOnBox = false;
         double deltaXEndmin = boundingBoxMinX - endX;
         double deltaXEndmax = boundingBoxMaxX - endX;
         double deltaYEndmin = boundingBoxMinY - endY;
         double deltaYEndmax = boundingBoxMaxY - endY;
         double deltaZEndmin = boundingBoxMinZ - endZ;
         double deltaZEndmax = boundingBoxMaxZ - endZ;

         if (isColinearX)
         {// check if start point of line-segment lies on bounding box surface   

            if (deltaYmin <= 0.0 && deltaYmax >= 0.0 && deltaZmin <= 0.0 && deltaZmax >= 0.0)
            { // line-segment start point lies on x surface of box
               isLineSegmentStartOnBox = true;
            }
            if (deltaYEndmin <= 0.0 && deltaYEndmax >= 0.0 && deltaZEndmin <= 0.0 && deltaZEndmax >= 0.0)
            { // line-segment end point also lies on x surface of box
               isLineSegmentEndOnBox = true;
            }

         }
         else if (isColinearY)
         {// check if start point of ray lies on bounding box surface

            if (deltaXmin <= 0.0 && deltaXmax >= 0.0 && deltaZmin <= 0.0 && deltaZmax >= 0.0)
            { // line-segment start point lies on x surface of box
               isLineSegmentStartOnBox = true;
            }
            if (deltaXEndmin <= 0.0 && deltaXEndmax >= 0.0 && deltaZEndmin <= 0.0 && deltaZEndmax >= 0.0)
            { // line-segment end point also lies on x surface of box
               isLineSegmentEndOnBox = true;

            }
         }
         else if (isColinearZ)
         {// check if start point of ray lies on bounding box surface

            if (deltaYmin <= 0.0 && deltaYmax >= 0.0 && deltaXmin <= 0.0 && deltaXmax >= 0.0)
            { // line-segment start point lies on x surface of box
               isLineSegmentStartOnBox = true;
            }
            if (deltaYEndmin <= 0.0 && deltaYEndmax >= 0.0 && deltaXEndmin <= 0.0 && deltaXEndmax >= 0.0)
            { // line-segment end point also lies on x surface of box
               isLineSegmentEndOnBox = true;
            }

         }
         if (isLineSegmentStartOnBox && isLineSegmentEndOnBox)
         { // start and end are on the box, we consider them intersections
            if (firstIntersectionToPack != null)
            {
               firstIntersectionToPack.set(startX, startY, startZ);
               tmin = 0.0;
            }
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(endX, endY, endZ);
               tmax = 1.0;
            }
            numberOfIntersections = 2;
         }
         else if (isLineSegmentStartOnBox)
         { // only the  start is on the box, we consider the segment start as the first intersection
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(firstIntersectionToPack);
            }
            if (firstIntersectionToPack != null)
            {
               firstIntersectionToPack.set(startX, startY, startZ);
               tmin = 0.0;
            }
            numberOfIntersections = 2;
         }
         else
         { // only the  end is on the box, we consider the segment end as the first intersection
            if (secondIntersectionToPack != null)
            {
               secondIntersectionToPack.set(endX, endY, endZ);
               tmax = 1.0;
            }
            numberOfIntersections = 2;
         }
      }

      // check if the two intersection points are (almost) identical
      if (numberOfIntersections == 2 && EuclidCoreTools.epsilonEquals(tmin, tmax, ONE_TRILLIONTH))
      {
         numberOfIntersections = 1;

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.setToNaN();
         }
      }

      return numberOfIntersections;

   }

   /**
    * Computes the coordinates of the possible intersections between a line3D and a box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the box are not restricted to exist between the two given
    * points defining the line.
    * <p>
    * In the case the line and the box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line is on a face of the box, the points where the line first/last intersects with the
    * box are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boxPosition              the coordinates of the box position. Not modified.
    * @param boxOrientation           the orientation of the box. Not modified.
    * @param boxSize                  the size of the box. Not modified.
    * @param startX                   the X coordinate of the first point defining the line. Not
    *                                 modified.
    * @param startY                   the Y coordinate of the first point defining the line. Not
    *                                 modified.
    * @param startZ                   the Z coordinate of the first point defining the line. Not
    *                                 modified.
    * @param endX                     the X coordinate of the second point defining the line. Not
    *                                 modified.
    * @param endY                     the Y coordinate of the second point defining the line. Not
    *                                 modified.
    * @param endZ                     the Z coordinate of the second point defining the line. Not
    *                                 modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the 3D box. It is either equal to 0, 1,
    *         or 2.
    * @throws IllegalArgumentException if {@code boxSize} contains values <= 0.0
    */
   public static int intersectionBetweenLine3DAndBox3D(Point3DReadOnly boxPosition,
                                                       Orientation3DReadOnly boxOrientation,
                                                       Vector3DReadOnly boxSize,

                                                       double startX,
                                                       double startY,
                                                       double startZ,
                                                       double endX,
                                                       double endY,
                                                       double endZ,

                                                       Point3DBasics firstIntersectionToPack,
                                                       Point3DBasics secondIntersectionToPack)
   {
      if (boxSize.getX() <= 0.0 || boxSize.getY() <= 0.0 || boxSize.getZ() <= 0.0)
         throw new IllegalArgumentException("The box size has to be positive and bigger 0.");

      // if arguments null:
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();

      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      if (firstIntersectionToPack == null)
         firstIntersectionToPack = new Point3D();
      if (secondIntersectionToPack == null)
         secondIntersectionToPack = new Point3D();

      firstIntersectionToPack.set(startX, startY, startZ);
      firstIntersectionToPack.sub(boxPosition);
      boxOrientation.inverseTransform(firstIntersectionToPack);

      double firstPointOnLineX = firstIntersectionToPack.getX();
      double firstPointOnLineY = firstIntersectionToPack.getY();
      double firstPointOnLineZ = firstIntersectionToPack.getZ();

      // construct the second point on line
      secondIntersectionToPack.set(endX, endY, endZ);
      secondIntersectionToPack.sub(boxPosition);
      boxOrientation.inverseTransform(secondIntersectionToPack);

      double secondPointOnLineX = secondIntersectionToPack.getX();
      double secondPointOnLineY = secondIntersectionToPack.getY();
      double secondPointOnLineZ = secondIntersectionToPack.getZ();

      double boundingBoxMinX = -0.5 * boxSize.getX();
      double boundingBoxMinY = -0.5 * boxSize.getY();
      double boundingBoxMinZ = -0.5 * boxSize.getZ();
      double boundingBoxMaxX = 0.5 * boxSize.getX();
      double boundingBoxMaxY = 0.5 * boxSize.getY();
      double boundingBoxMaxZ = 0.5 * boxSize.getZ();

      int numIntersections = intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMinX,
                                                                           boundingBoxMinY,
                                                                           boundingBoxMinZ,
                                                                           boundingBoxMaxX,
                                                                           boundingBoxMaxY,
                                                                           boundingBoxMaxZ,
                                                                           firstPointOnLineX,
                                                                           firstPointOnLineY,
                                                                           firstPointOnLineZ,
                                                                           true,
                                                                           secondPointOnLineX,
                                                                           secondPointOnLineY,
                                                                           secondPointOnLineZ,
                                                                           true,
                                                                           firstIntersectionToPack,
                                                                           secondIntersectionToPack);

      boxOrientation.transform(firstIntersectionToPack);
      firstIntersectionToPack.add(boxPosition);
      boxOrientation.transform(secondIntersectionToPack);
      secondIntersectionToPack.add(boxPosition);

      return numIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPositionX        the x-coordinate of the center of the cylinder.
    * @param cylinderPositionY        the y-coordinate of the center of the cylinder.
    * @param cylinderPositionZ        the z-coordinate of the center of the cylinder.
    * @param cylinderAxisX            the x-component of the cylinder's axis.
    * @param cylinderAxisY            the y-component of the cylinder's axis.
    * @param cylinderAxisZ            the z-component of the cylinder's axis.
    * @param pointOnLineX             the x-coordinate of a point located on the infinitely long line.
    * @param pointOnLineY             the y-coordinate of a point located on the infinitely long line.
    * @param pointOnLineZ             the z-coordinate of a point located on the infinitely long line.
    * @param lineDirectionX           the x-component of the direction of the line.
    * @param lineDirectionY           the y-component of the direction of the line.
    * @param lineDirectionZ           the z-component of the direction of the line.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException if either {@code cylinderLength < 0} or
    *                                  {@code cylinderRadius < 0}.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            double cylinderPositionX,
                                                            double cylinderPositionY,
                                                            double cylinderPositionZ,
                                                            double cylinderAxisX,
                                                            double cylinderAxisY,
                                                            double cylinderAxisZ,
                                                            double pointOnLineX,
                                                            double pointOnLineY,
                                                            double pointOnLineZ,
                                                            double lineDirectionX,
                                                            double lineDirectionY,
                                                            double lineDirectionZ,
                                                            Point3DBasics firstIntersectionToPack,
                                                            Point3DBasics secondIntersectionToPack)
   {
      double startX = pointOnLineX;
      double startY = pointOnLineY;
      double startZ = pointOnLineZ;
      double endX = pointOnLineX + lineDirectionX;
      double endY = pointOnLineY + lineDirectionY;
      double endZ = pointOnLineZ + lineDirectionZ;
      return intersectionBetweenLine3DAndCylinder3DImpl(cylinderLength,
                                                        cylinderRadius,
                                                        cylinderPositionX,
                                                        cylinderPositionY,
                                                        cylinderPositionZ,
                                                        cylinderAxisX,
                                                        cylinderAxisY,
                                                        cylinderAxisZ,
                                                        false,
                                                        startX,
                                                        startY,
                                                        startZ,
                                                        true,
                                                        endX,
                                                        endY,
                                                        endZ,
                                                        true,
                                                        firstIntersectionToPack,
                                                        secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException if either {@code cylinderLength < 0} or
    *                                  {@code cylinderRadius < 0}.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            Point3DReadOnly cylinderPosition,
                                                            Vector3DReadOnly cylinderAxis,
                                                            Point3DReadOnly firstPointOnLine,
                                                            Point3DReadOnly secondPointOnLine,
                                                            Point3DBasics firstIntersectionToPack,
                                                            Point3DBasics secondIntersectionToPack)
   {
      double cylinderPositionX = cylinderPosition.getX();
      double cylinderPositionY = cylinderPosition.getY();
      double cylinderPositionZ = cylinderPosition.getZ();
      double cylinderAxisX = cylinderAxis.getX();
      double cylinderAxisY = cylinderAxis.getY();
      double cylinderAxisZ = cylinderAxis.getZ();
      double startX = firstPointOnLine.getX();
      double startY = firstPointOnLine.getY();
      double startZ = firstPointOnLine.getZ();
      double endX = secondPointOnLine.getX();
      double endY = secondPointOnLine.getY();
      double endZ = secondPointOnLine.getZ();
      return intersectionBetweenLine3DAndCylinder3DImpl(cylinderLength,
                                                        cylinderRadius,
                                                        cylinderPositionX,
                                                        cylinderPositionY,
                                                        cylinderPositionZ,
                                                        cylinderAxisX,
                                                        cylinderAxisY,
                                                        cylinderAxisZ,
                                                        cylinderAxis instanceof UnitVector3DReadOnly,
                                                        startX,
                                                        startY,
                                                        startZ,
                                                        true,
                                                        endX,
                                                        endY,
                                                        endZ,
                                                        true,
                                                        firstIntersectionToPack,
                                                        secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException if either {@code cylinderLength < 0} or
    *                                  {@code cylinderRadius < 0}.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            Point3DReadOnly cylinderPosition,
                                                            Vector3DReadOnly cylinderAxis,
                                                            Point3DReadOnly pointOnLine,
                                                            Vector3DReadOnly lineDirection,
                                                            Point3DBasics firstIntersectionToPack,
                                                            Point3DBasics secondIntersectionToPack)
   {
      double cylinderPositionX = cylinderPosition.getX();
      double cylinderPositionY = cylinderPosition.getY();
      double cylinderPositionZ = cylinderPosition.getZ();
      double cylinderAxisX = cylinderAxis.getX();
      double cylinderAxisY = cylinderAxis.getY();
      double cylinderAxisZ = cylinderAxis.getZ();
      double startX = pointOnLine.getX();
      double startY = pointOnLine.getY();
      double startZ = pointOnLine.getZ();
      double endX = pointOnLine.getX() + lineDirection.getX();
      double endY = pointOnLine.getY() + lineDirection.getY();
      double endZ = pointOnLine.getZ() + lineDirection.getZ();
      return intersectionBetweenLine3DAndCylinder3DImpl(cylinderLength,
                                                        cylinderRadius,
                                                        cylinderPositionX,
                                                        cylinderPositionY,
                                                        cylinderPositionZ,
                                                        cylinderAxisX,
                                                        cylinderAxisY,
                                                        cylinderAxisZ,
                                                        cylinderAxis instanceof UnitVector3DReadOnly,
                                                        startX,
                                                        startY,
                                                        startZ,
                                                        true,
                                                        endX,
                                                        endY,
                                                        endZ,
                                                        true,
                                                        firstIntersectionToPack,
                                                        secondIntersectionToPack);
   }

   /**
    * Flexible implementation for computing the intersection between a cylinder and either a line, a
    * line segment, or a ray.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * Switching between line/line-segment/ray can be done using the two arguments
    * {@code canIntersectionOccurBeforeStart} and {@code canIntersectionOccurAfterEnd}:
    * <ul>
    * <li>{@code canIntersectionOccurBeforeStart == true} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate line/cylinder
    * intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == false} changes the algorithm to calculate
    * line-segment/cylinder intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate ray/cylinder
    * intersection.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength                  length of the cylinder.
    * @param cylinderRadius                  radius of the cylinder.
    * @param cylinderPositionX               the x-coordinate of the center of the cylinder.
    * @param cylinderPositionY               the y-coordinate of the center of the cylinder.
    * @param cylinderPositionZ               the z-coordinate of the center of the cylinder.
    * @param cylinderAxisX                   the x-component of the cylinder's axis.
    * @param cylinderAxisY                   the y-component of the cylinder's axis.
    * @param cylinderAxisZ                   the z-component of the cylinder's axis.
    * @param isAxisUnitary                   whether the cylinder's axis is already normalized.
    * @param startX                          the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startY                          the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startZ                          the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart specifies whether an intersection can exist before
    *                                        {@code start}.
    * @param endX                            the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endY                            the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endZ                            the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurAfterEnd    specifies whether an intersection can exist after
    *                                        {@code end}.
    * @param firstIntersectionToPack         the coordinate of the first intersection. Can be
    *                                        {@code null}. Modified.
    * @param secondIntersectionToPack        the coordinate of the second intersection. Can be
    *                                        {@code null}. Modified.
    * @return the number of intersections between the line/line-segment/ray and the cylinder. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code cylinderLength < 0} or
    *                                  {@code cylinderRadius < 0}.
    */
   private static int intersectionBetweenLine3DAndCylinder3DImpl(double cylinderLength,
                                                                 double cylinderRadius,
                                                                 double cylinderPositionX,
                                                                 double cylinderPositionY,
                                                                 double cylinderPositionZ,
                                                                 double cylinderAxisX,
                                                                 double cylinderAxisY,
                                                                 double cylinderAxisZ,
                                                                 boolean isAxisUnitary,
                                                                 double startX,
                                                                 double startY,
                                                                 double startZ,
                                                                 boolean canIntersectionOccurBeforeStart,
                                                                 double endX,
                                                                 double endY,
                                                                 double endZ,
                                                                 boolean canIntersectionOccurAfterEnd,
                                                                 Point3DBasics firstIntersectionToPack,
                                                                 Point3DBasics secondIntersectionToPack)
   {
      if (cylinderLength < 0.0)
         throw new IllegalArgumentException("The cylinder length has to be positive.");
      if (cylinderRadius < 0.0)
         throw new IllegalArgumentException("The cylinder radius has to be positive.");

      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      if (cylinderLength == 0.0 || cylinderRadius == 0.0)
         return 0;

      double axisNormInv = isAxisUnitary ? 1.0 : 1.0 / EuclidCoreTools.fastNorm(cylinderAxisX, cylinderAxisY, cylinderAxisZ);
      double axisX = cylinderAxisX * axisNormInv;
      double axisY = cylinderAxisY * axisNormInv;
      double axisZ = cylinderAxisZ * axisNormInv;

      double halfLength = 0.5 * cylinderLength;
      double radiusSquared = cylinderRadius * cylinderRadius;

      double dIntersection1 = Double.NaN;
      double dIntersection2 = Double.NaN;

      double dx = endX - startX;
      double dy = endY - startY;
      double dz = endZ - startZ;

      double topX = halfLength * axisX + cylinderPositionX;
      double topY = halfLength * axisY + cylinderPositionY;
      double topZ = halfLength * axisZ + cylinderPositionZ;

      double bottomX = -halfLength * axisX + cylinderPositionX;
      double bottomY = -halfLength * axisY + cylinderPositionY;
      double bottomZ = -halfLength * axisZ + cylinderPositionZ;
      double lineDirectionDotCylinderAxis = dx * axisX + dy * axisY + dz * axisZ;

      if (Math.abs(lineDirectionDotCylinderAxis) >= ONE_TRILLIONTH)
      {
         double dTop = Double.NaN;
         { // Compute the intersection with the top face using line-plane intersection.
            double numerator = (topX - startX) * axisX + (topY - startY) * axisY + (topZ - startZ) * axisZ;
            dTop = numerator / lineDirectionDotCylinderAxis;
            double intersectionX = dTop * dx + startX;
            double intersectionY = dTop * dy + startY;
            double intersectionZ = dTop * dz + startZ;
            if (distanceSquaredBetweenPoint3Ds(intersectionX, intersectionY, intersectionZ, topX, topY, topZ) > radiusSquared)
               dTop = Double.NaN;
         }

         if (Double.isFinite(dTop))
            dIntersection1 = dTop;

         double dBottom = Double.NaN;
         { // Compute the intersection with the bottom face using simplified line-plane intersection.
            double numerator = (bottomX - startX) * axisX + (bottomY - startY) * axisY + (bottomZ - startZ) * axisZ;
            dBottom = numerator / lineDirectionDotCylinderAxis;
            double intersectionX = dBottom * dx + startX;
            double intersectionY = dBottom * dy + startY;
            double intersectionZ = dBottom * dz + startZ;
            if (distanceSquaredBetweenPoint3Ds(intersectionX, intersectionY, intersectionZ, bottomX, bottomY, bottomZ) > radiusSquared)
               dBottom = Double.NaN;
         }

         if (Double.isFinite(dBottom))
         {
            if (Double.isNaN(dIntersection1))
            {
               dIntersection1 = dBottom;
            }
            else if (dBottom < dIntersection1)
            {
               dIntersection2 = dIntersection1;
               dIntersection1 = dBottom;
            }
            else
            {
               dIntersection2 = dBottom;
            }
         }
      }

      // If dIntersection2 is not NaN, that means two intersections were found which is the max, so no need to check with the cylinder part.
      if (Double.isNaN(dIntersection2))
      { // Compute possible intersections with the cylinder part
        // Notation used: cylinder axis: pa + va * d; line equation: p + v * d
        // Need to solve quadratic equation of the form A * d^2 + B * d + C = 0

         // (v, va)*va
         double scaledAxisX1 = lineDirectionDotCylinderAxis * axisX;
         double scaledAxisY1 = lineDirectionDotCylinderAxis * axisY;
         double scaledAxisZ1 = lineDirectionDotCylinderAxis * axisZ;
         // Vector used for computing A and B: v - (v, va)*va
         double AX = dx - scaledAxisX1;
         double AY = dy - scaledAxisY1;
         double AZ = dz - scaledAxisZ1;
         // Delta_p
         double deltaPX = startX - cylinderPositionX;
         double deltaPY = startY - cylinderPositionY;
         double deltaPZ = startZ - cylinderPositionZ;
         // (Delta_p, va)
         double linePositionToCylinderDotAxis = deltaPX * axisX + deltaPY * axisY + deltaPZ * axisZ;
         // (Delta_p, va)*va
         double scaledAxisX2 = linePositionToCylinderDotAxis * axisX;
         double scaledAxisY2 = linePositionToCylinderDotAxis * axisY;
         double scaledAxisZ2 = linePositionToCylinderDotAxis * axisZ;
         // Vector used for computing B and C: Delta_p - (Delta_p, va)*va
         double CX = deltaPX - scaledAxisX2;
         double CY = deltaPY - scaledAxisY2;
         double CZ = deltaPZ - scaledAxisZ2;

         // A = (v - (v, va)*va)^2
         double A = EuclidCoreTools.normSquared(AX, AY, AZ);
         // B = 2*(v - (v, va)*va, Delta_p - (Delta_p, va)*va)
         double B = 2.0 * (AX * CX + AY * CY + AZ * CZ);
         // C = (p - (Delta_p, va)*va)^2 - r^2
         double C = EuclidCoreTools.normSquared(CX, CY, CZ) - radiusSquared;

         double delta = EuclidCoreTools.squareRoot(B * B - 4 * A * C);

         if (Double.isFinite(delta))
         {
            double oneOverTwoA = 0.5 / A;
            double dCylinder1 = (-B + delta) * oneOverTwoA;
            double dCylinder2 = (-B - delta) * oneOverTwoA;

            double intersection1X = dCylinder1 * dx + startX;
            double intersection1Y = dCylinder1 * dy + startY;
            double intersection1Z = dCylinder1 * dz + startZ;
            if (Math.abs(percentageAlongLine3D(intersection1X,
                                               intersection1Y,
                                               intersection1Z,
                                               cylinderPositionX,
                                               cylinderPositionY,
                                               cylinderPositionZ,
                                               axisX,
                                               axisY,
                                               axisZ))
                > halfLength - ONE_TRILLIONTH)
               dCylinder1 = Double.NaN;

            if (Double.isFinite(dCylinder1))
            {
               if (Double.isNaN(dIntersection1) || Math.abs(dCylinder1 - dIntersection1) < ONE_TRILLIONTH)
               {
                  dIntersection1 = dCylinder1;
               }
               else if (dCylinder1 < dIntersection1)
               {
                  dIntersection2 = dIntersection1;
                  dIntersection1 = dCylinder1;
               }
               else
               {
                  dIntersection2 = dCylinder1;
               }
            }

            double intersection2X = dCylinder2 * dx + startX;
            double intersection2Y = dCylinder2 * dy + startY;
            double intersection2Z = dCylinder2 * dz + startZ;
            if (Math.abs(percentageAlongLine3D(intersection2X,
                                               intersection2Y,
                                               intersection2Z,
                                               cylinderPositionX,
                                               cylinderPositionY,
                                               cylinderPositionZ,
                                               axisX,
                                               axisY,
                                               axisZ))
                > halfLength - ONE_TRILLIONTH)
               dCylinder2 = Double.NaN;
            else if (Math.abs(dCylinder1 - dCylinder2) < ONE_TRILLIONTH)
               dCylinder2 = Double.NaN;

            if (Double.isFinite(dCylinder2))
            {
               if (Double.isNaN(dIntersection1))
               {
                  dIntersection1 = dCylinder2;
               }
               else if (dCylinder2 < dIntersection1)
               {
                  dIntersection2 = dIntersection1;
                  dIntersection1 = dCylinder2;
               }
               else
               {
                  dIntersection2 = dCylinder2;
               }
            }
         }
      }

      if (!canIntersectionOccurBeforeStart)
      {
         if (dIntersection2 < 0.0)
            dIntersection2 = Double.NaN;

         if (dIntersection1 < 0.0)
         {
            dIntersection1 = dIntersection2;
            dIntersection2 = Double.NaN;
         }
      }

      if (!canIntersectionOccurAfterEnd)
      {
         if (dIntersection2 > 1.0)
            dIntersection2 = Double.NaN;

         if (dIntersection1 > 1.0)
         {
            dIntersection1 = dIntersection2;
            dIntersection2 = Double.NaN;
         }
      }

      if (Double.isNaN(dIntersection1))
         return 0;

      if (firstIntersectionToPack != null)
      {
         firstIntersectionToPack.set(dx, dy, dz);
         firstIntersectionToPack.scale(dIntersection1);
         firstIntersectionToPack.add(startX, startY, startZ);
      }

      if (Double.isNaN(dIntersection2))
         return 1;

      if (secondIntersectionToPack != null)
      {
         secondIntersectionToPack.set(dx, dy, dz);
         secondIntersectionToPack.scale(dIntersection2);
         secondIntersectionToPack.add(startX, startY, startZ);
      }

      return 2;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param pointOnLineX             the x-coordinate of a point located on the infinitely long line.
    * @param pointOnLineY             the y-coordinate of a point located on the infinitely long line.
    * @param pointOnLineZ             the z-coordinate of a point located on the infinitely long line.
    * @param lineDirectionX           the x-component of the direction of the line.
    * @param lineDirectionY           the y-component of the direction of the line.
    * @param lineDirectionZ           the z-component of the direction of the line.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code radiusX}, {@code radiusY}, or {@code radiusZ}
    *                                  is negative.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             double pointOnLineX,
                                                             double pointOnLineY,
                                                             double pointOnLineZ,
                                                             double lineDirectionX,
                                                             double lineDirectionY,
                                                             double lineDirectionZ,
                                                             Point3DBasics firstIntersectionToPack,
                                                             Point3DBasics secondIntersectionToPack)
   {
      double startX = pointOnLineX;
      double startY = pointOnLineY;
      double startZ = pointOnLineZ;
      double endX = pointOnLineX + lineDirectionX;
      double endY = pointOnLineY + lineDirectionY;
      double endZ = pointOnLineZ + lineDirectionZ;
      return intersectionBetweenLine3DAndEllipsoid3DImpl(radiusX,
                                                         radiusY,
                                                         radiusZ,
                                                         startX,
                                                         startY,
                                                         startZ,
                                                         true,
                                                         endX,
                                                         endY,
                                                         endZ,
                                                         true,
                                                         firstIntersectionToPack,
                                                         secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code radiusX}, {@code radiusY}, or {@code radiusZ}
    *                                  is negative.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             Point3DReadOnly firstPointOnLine,
                                                             Point3DReadOnly secondPointOnLine,
                                                             Point3DBasics firstIntersectionToPack,
                                                             Point3DBasics secondIntersectionToPack)
   {
      double startX = firstPointOnLine.getX();
      double startY = firstPointOnLine.getY();
      double startZ = firstPointOnLine.getZ();
      double endX = secondPointOnLine.getX();
      double endY = secondPointOnLine.getY();
      double endZ = secondPointOnLine.getZ();
      return intersectionBetweenLine3DAndEllipsoid3DImpl(radiusX,
                                                         radiusY,
                                                         radiusZ,
                                                         startX,
                                                         startY,
                                                         startZ,
                                                         true,
                                                         endX,
                                                         endY,
                                                         endZ,
                                                         true,
                                                         firstIntersectionToPack,
                                                         secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code radiusX}, {@code radiusY}, or {@code radiusZ}
    *                                  is negative.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             Point3DReadOnly pointOnLine,
                                                             Vector3DReadOnly lineDirection,
                                                             Point3DBasics firstIntersectionToPack,
                                                             Point3DBasics secondIntersectionToPack)
   {
      double startX = pointOnLine.getX();
      double startY = pointOnLine.getY();
      double startZ = pointOnLine.getZ();
      double endX = pointOnLine.getX() + lineDirection.getX();
      double endY = pointOnLine.getY() + lineDirection.getY();
      double endZ = pointOnLine.getZ() + lineDirection.getZ();
      return intersectionBetweenLine3DAndEllipsoid3DImpl(radiusX,
                                                         radiusY,
                                                         radiusZ,
                                                         startX,
                                                         startY,
                                                         startZ,
                                                         true,
                                                         endX,
                                                         endY,
                                                         endZ,
                                                         true,
                                                         firstIntersectionToPack,
                                                         secondIntersectionToPack);
   }

   /**
    * Flexible implementation for computing the intersection between an ellipsoid and either a line, a
    * line segment, or a ray.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * Switching between line/line-segment/ray can be done using the two arguments
    * {@code canIntersectionOccurBeforeStart} and {@code canIntersectionOccurAfterEnd}:
    * <ul>
    * <li>{@code canIntersectionOccurBeforeStart == true} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate line/ellipsoid
    * intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == false} changes the algorithm to calculate
    * line-segment/ellipsoid intersection.
    * <li>{@code canIntersectionOccurBeforeStart == false} and
    * {@code canIntersectionOccurAfterEnd == true} changes the algorithm to calculate ray/ellipsoid
    * intersection.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                         radius of the ellipsoid along the x-axis.
    * @param radiusY                         radius of the ellipsoid along the y-axis.
    * @param radiusZ                         radius of the ellipsoid along the z-axis.
    * @param startX                          the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startY                          the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param startZ                          the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurBeforeStart specifies whether an intersection can exist before
    *                                        {@code start}.
    * @param endX                            the x-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endY                            the y-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param endZ                            the z-coordinate of a point located on the
    *                                        line/line-segment/ray.
    * @param canIntersectionOccurAfterEnd    specifies whether an intersection can exist after
    *                                        {@code end}.
    * @param firstIntersectionToPack         the coordinate of the first intersection. Can be
    *                                        {@code null}. Modified.
    * @param secondIntersectionToPack        the coordinate of the second intersection. Can be
    *                                        {@code null}. Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code radiusX}, {@code radiusY}, or {@code radiusZ}
    *                                  is negative.
    */
   private static int intersectionBetweenLine3DAndEllipsoid3DImpl(double radiusX,
                                                                  double radiusY,
                                                                  double radiusZ,
                                                                  double startX,
                                                                  double startY,
                                                                  double startZ,
                                                                  boolean canIntersectionOccurBeforeStart,
                                                                  double endX,
                                                                  double endY,
                                                                  double endZ,
                                                                  boolean canIntersectionOccurAfterEnd,
                                                                  Point3DBasics firstIntersectionToPack,
                                                                  Point3DBasics secondIntersectionToPack)
   {
      if (radiusX < 0.0)
         throw new IllegalArgumentException("The ellipsoid x-radius has to be positive.");
      if (radiusY < 0.0)
         throw new IllegalArgumentException("The ellipsoid y-radius has to be positive.");
      if (radiusZ < 0.0)
         throw new IllegalArgumentException("The ellipsoid z-radius has to be positive.");

      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      if (radiusX == 0.0 || radiusY == 0.0 || radiusZ == 0.0)
         return 0;

      double dx = endX - startX;
      double dy = endY - startY;
      double dz = endZ - startZ;

      // The equation for an ellipsoid is: x^2/rx^2 + y^2/ry^2 + z^2/rz^2 = 1
      // Plugging in the equation of the point p that belongs to a line: p = (x, y, z) = x0 + v t
      // We can write a quadratic equation in t: A t^2 + B t + C = 0
      // To simplify, let's write p' = (x/rx, y/ry, z/rz)
      double x = startX / radiusX;
      double y = startY / radiusY;
      double z = startZ / radiusZ;

      double vx = dx / radiusX;
      double vy = dy / radiusY;
      double vz = dz / radiusZ;

      // By plugging the line equation in the ellipsoid equation we can find A, B, and C
      // A = vx^2/rx^2 + vy^2/ry^2 + vz^2/rz^2
      double A = EuclidCoreTools.normSquared(vx, vy, vz);
      // B = 2 (x*vx/rx^2 + y*vy/ry^2 + z*vz/rz^2)
      double B = 2.0 * (x * vx + y * vy + z * vz);
      // C = x^2/rx^2 + y^2/ry^2 + z^2/rz^2 - 1
      double C = EuclidCoreTools.normSquared(x, y, z) - 1.0;

      double delta = B * B - 4.0 * A * C;

      if (delta < 0.0)
         return 0;

      double oneOverTwoA = 1.0 / (2.0 * A);
      delta = EuclidCoreTools.squareRoot(delta);

      double t1, t2;

      if (delta < ONE_TRILLIONTH)
      {
         t1 = -B * oneOverTwoA;
         t2 = Double.NaN;
      }
      else
      {
         t1 = (-B - delta) * oneOverTwoA;
         t2 = (-B + delta) * oneOverTwoA;
      }

      if (!canIntersectionOccurBeforeStart)
      {
         if (t2 < 0.0)
            t2 = Double.NaN;

         if (t1 < 0.0)
         {
            t1 = t2;
            t2 = Double.NaN;
         }
      }

      if (!canIntersectionOccurAfterEnd)
      {
         if (t2 > 1.0)
            t2 = Double.NaN;

         if (t1 > 1.0)
         {
            t1 = t2;
            t2 = Double.NaN;
         }
      }

      if (Double.isNaN(t1))
         return 0;

      if (firstIntersectionToPack != null)
      {
         firstIntersectionToPack.set(dx, dy, dz);
         firstIntersectionToPack.scale(t1);
         firstIntersectionToPack.add(startX, startY, startZ);
      }

      if (Double.isNaN(t2))
         return 1;

      if (secondIntersectionToPack != null)
      {
         secondIntersectionToPack.set(dx, dy, dz);
         secondIntersectionToPack.scale(t2);
         secondIntersectionToPack.add(startX, startY, startZ);
      }

      return 2;
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane  a point located on the plane. Not modified.
    * @param planeNormal   the normal of the plane. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the coordinates of the intersection, or {@code null} if the line is parallel to the
    *         plane.
    */
   public static Point3D intersectionBetweenLine3DAndPlane3D(Point3DReadOnly pointOnPlane,
                                                             Vector3DReadOnly planeNormal,
                                                             Point3DReadOnly pointOnLine,
                                                             Vector3DReadOnly lineDirection)
   {
      Point3D intersection = new Point3D();
      boolean success = intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection, intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeNormal        the normal of the plane. Not modified.
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the direction of the line. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    */
   public static boolean intersectionBetweenLine3DAndPlane3D(Point3DReadOnly pointOnPlane,
                                                             Vector3DReadOnly planeNormal,
                                                             Point3DReadOnly pointOnLine,
                                                             Vector3DReadOnly lineDirection,
                                                             Point3DBasics intersectionToPack)
   {
      // Switching to the notation used in https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
      // Note: the algorithm is independent from the magnitudes of planeNormal and lineDirection
      Point3DReadOnly p0 = pointOnPlane;
      Vector3DReadOnly n = planeNormal;
      Point3DReadOnly l0 = pointOnLine;
      Vector3DReadOnly l = lineDirection;

      // Let's compute the value of the coefficient d = ( (p0 - l0).n ) / ( l.n )
      double d, numerator, denominator;
      numerator = (p0.getX() - l0.getX()) * n.getX();
      numerator += (p0.getY() - l0.getY()) * n.getY();
      numerator += (p0.getZ() - l0.getZ()) * n.getZ();
      denominator = l.dot(n);

      // Check if the line is parallel to the plane
      if (Math.abs(denominator) < ONE_TRILLIONTH)
      {
         return false;
      }
      else
      {
         d = numerator / denominator;

         intersectionToPack.scaleAdd(d, l, l0);
         return true;
      }
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and the bounding box do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line segment start and/or end point lie on the boundary of the bounding box they are
    * considered as intersection points.
    * <li>if a line segment is colinear with a boundary of the bounding box, the points where the line
    * first/last intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLineSegment2DAndBoundingBox2D(Point2DReadOnly boundingBoxMin,
                                                                      Point2DReadOnly boundingBoxMax,
                                                                      Point2DReadOnly lineSegmentStart,
                                                                      Point2DReadOnly lineSegmentEnd,
                                                                      Point2DBasics firstIntersectionToPack,
                                                                      Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine2DAndBoundingBox2DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           lineSegmentStart.getX(),
                                                           lineSegmentStart.getY(),
                                                           false,
                                                           lineSegmentEnd.getX(),
                                                           lineSegmentEnd.getY(),
                                                           false,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and the bounding box do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the start or end point of the line segment lie on the boundary of the bounding box they
    * are considered as intersection points.
    * <li>if a line segment is colinear with a boundary of the bounding box, the points where the line
    * first/last intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenLineSegment3DAndBoundingBox3D(Point3DReadOnly boundingBoxMin,
                                                                      Point3DReadOnly boundingBoxMax,
                                                                      Point3DReadOnly lineSegmentStart,
                                                                      Point3DReadOnly lineSegmentEnd,
                                                                      Point3DBasics firstIntersectionToPack,
                                                                      Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           lineSegmentStart.getX(),
                                                           lineSegmentStart.getY(),
                                                           lineSegmentStart.getZ(),
                                                           false,
                                                           lineSegmentEnd.getX(),
                                                           lineSegmentEnd.getY(),
                                                           lineSegmentEnd.getZ(),
                                                           false,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and a box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the box can only exist between the endpoints of the
    * line segment.
    * </p>
    * <p>
    * In the case the line segment and the box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the start or end point of the line segment lie on the boundary of the box they are
    * considered as intersection points.
    * <li>if the line segment is on one of the box's faces, the points where the line segment
    * first/last intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    * 
    * @param boxPosition              the coordinates of the box position. Not modified.
    * @param boxOrientation           the orientation of the box. Not modified.
    * @param boxSize                  the size of the box. Not modified.
    * @param startX                   the X coordinate of the first point defining the line segment.
    *                                 Not modified.
    * @param startY                   the Y coordinate of the first point defining the line segment.
    *                                 Not modified.
    * @param startZ                   the Z coordinate of the first point defining the line segment.
    *                                 Not modified.
    * @param endX                     the X coordinate of the second point defining the line segment.
    *                                 Not modified.
    * @param endY                     the Y coordinate of the second point defining the line segment.
    *                                 Not modified.
    * @param endZ                     the Z coordinate of the second point defining the line segment.
    *                                 Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the 3D box. It is either equal
    *         to 0, 1, or 2. If the segment origin is on the surface of the 3D box it is considered an
    *         intersection.
    * @throws IllegalArgumentException if {@code boxSize} contains values <= 0.0
    */
   public static int intersectionBetweenLineSegment3DAndBox3D(Point3DReadOnly boxPosition,
                                                              Orientation3DReadOnly boxOrientation,
                                                              Vector3DReadOnly boxSize,

                                                              double startX,
                                                              double startY,
                                                              double startZ,
                                                              double endX,
                                                              double endY,
                                                              double endZ,

                                                              Point3DBasics firstIntersectionToPack,
                                                              Point3DBasics secondIntersectionToPack)
   {
      if (boxSize.getX() <= 0.0 || boxSize.getY() <= 0.0 || boxSize.getZ() <= 0.0)
         throw new IllegalArgumentException("The box size has to be positive and bigger 0.");

      // if arguments null:
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      if (firstIntersectionToPack == null)
         firstIntersectionToPack = new Point3D();
      if (secondIntersectionToPack == null)
         secondIntersectionToPack = new Point3D();

      firstIntersectionToPack.set(startX, startY, startZ);
      firstIntersectionToPack.sub(boxPosition);
      boxOrientation.inverseTransform(firstIntersectionToPack);

      double firstPointOnLineX = firstIntersectionToPack.getX();
      double firstPointOnLineY = firstIntersectionToPack.getY();
      double firstPointOnLineZ = firstIntersectionToPack.getZ();

      // construct the second point on line
      secondIntersectionToPack.set(endX, endY, endZ);
      secondIntersectionToPack.sub(boxPosition);
      boxOrientation.inverseTransform(secondIntersectionToPack);

      double secondPointOnLineX = secondIntersectionToPack.getX();
      double secondPointOnLineY = secondIntersectionToPack.getY();
      double secondPointOnLineZ = secondIntersectionToPack.getZ();

      double boundingBoxMinX = -0.5 * boxSize.getX();
      double boundingBoxMinY = -0.5 * boxSize.getY();
      double boundingBoxMinZ = -0.5 * boxSize.getZ();
      double boundingBoxMaxX = 0.5 * boxSize.getX();
      double boundingBoxMaxY = 0.5 * boxSize.getY();
      double boundingBoxMaxZ = 0.5 * boxSize.getZ();

      int numIntersections = intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMinX,
                                                                           boundingBoxMinY,
                                                                           boundingBoxMinZ,
                                                                           boundingBoxMaxX,
                                                                           boundingBoxMaxY,
                                                                           boundingBoxMaxZ,
                                                                           firstPointOnLineX,
                                                                           firstPointOnLineY,
                                                                           firstPointOnLineZ,
                                                                           false,
                                                                           secondPointOnLineX,
                                                                           secondPointOnLineY,
                                                                           secondPointOnLineZ,
                                                                           false,
                                                                           firstIntersectionToPack,
                                                                           secondIntersectionToPack);

      boxOrientation.transform(firstIntersectionToPack);
      firstIntersectionToPack.add(boxPosition);
      boxOrientation.transform(secondIntersectionToPack);
      secondIntersectionToPack.add(boxPosition);

      return numIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line segment and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the cylinder. It is either equal
    *         to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code cylinderLength < 0} or
    *                                  {@code cylinderRadius < 0}.
    */
   public static int intersectionBetweenLineSegment3DAndCylinder3D(double cylinderLength,
                                                                   double cylinderRadius,
                                                                   Point3DReadOnly cylinderPosition,
                                                                   Vector3DReadOnly cylinderAxis,
                                                                   Point3DReadOnly lineSegmentStart,
                                                                   Point3DReadOnly lineSegmentEnd,
                                                                   Point3DBasics firstIntersectionToPack,
                                                                   Point3DBasics secondIntersectionToPack)
   {
      double cylinderPositionX = cylinderPosition.getX();
      double cylinderPositionY = cylinderPosition.getY();
      double cylinderPositionZ = cylinderPosition.getZ();
      double cylinderAxisX = cylinderAxis.getX();
      double cylinderAxisY = cylinderAxis.getY();
      double cylinderAxisZ = cylinderAxis.getZ();
      double startX = lineSegmentStart.getX();
      double startY = lineSegmentStart.getY();
      double startZ = lineSegmentStart.getZ();
      double endX = lineSegmentEnd.getX();
      double endY = lineSegmentEnd.getY();
      double endZ = lineSegmentEnd.getZ();
      return intersectionBetweenLine3DAndCylinder3DImpl(cylinderLength,
                                                        cylinderRadius,
                                                        cylinderPositionX,
                                                        cylinderPositionY,
                                                        cylinderPositionZ,
                                                        cylinderAxisX,
                                                        cylinderAxisY,
                                                        cylinderAxisZ,
                                                        cylinderAxis instanceof UnitVector3DReadOnly,
                                                        startX,
                                                        startY,
                                                        startZ,
                                                        false,
                                                        endX,
                                                        endY,
                                                        endZ,
                                                        false,
                                                        firstIntersectionToPack,
                                                        secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line segment and the ellipsoid do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code radiusX}, {@code radiusY}, or {@code radiusZ}
    *                                  is negative.
    */
   public static int intersectionBetweenLineSegment3DAndEllipsoid3D(double radiusX,
                                                                    double radiusY,
                                                                    double radiusZ,
                                                                    Point3DReadOnly lineSegmentStart,
                                                                    Point3DReadOnly lineSegmentEnd,
                                                                    Point3DBasics firstIntersectionToPack,
                                                                    Point3DBasics secondIntersectionToPack)
   {
      double startX = lineSegmentStart.getX();
      double startY = lineSegmentStart.getY();
      double startZ = lineSegmentStart.getZ();
      double endX = lineSegmentEnd.getX();
      double endY = lineSegmentEnd.getY();
      double endZ = lineSegmentEnd.getZ();
      return intersectionBetweenLine3DAndEllipsoid3DImpl(radiusX,
                                                         radiusY,
                                                         radiusZ,
                                                         startX,
                                                         startY,
                                                         startZ,
                                                         false,
                                                         endX,
                                                         endY,
                                                         endZ,
                                                         false,
                                                         firstIntersectionToPack,
                                                         secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the intersection between a plane and a finite length line segment.
    * <p>
    * This method returns {@code null} for the following cases:
    * <ul>
    * <li>the line segment is parallel to the plane,
    * <li>the line segment endpoints are on one side of the plane,
    * <li>the line segment length is equal to zero ({@code lineSegmentStart == lineSegmentEnd}),
    * <li>one of the line segment endpoints lies on the plane.
    * </ul>
    * </p>
    * Once the existence of an intersection is verified, this method calls
    * {@link #intersectionBetweenLine3DAndPlane3D(Point3DReadOnly, Vector3DReadOnly, Point3DReadOnly, Vector3DReadOnly)}
    * to perform the actual computation.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnPlane     a point located on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the intersection, or {@code null} if there is no intersection.
    */
   public static Point3D intersectionBetweenLineSegment3DAndPlane3D(Point3DReadOnly pointOnPlane,
                                                                    Vector3DReadOnly planeNormal,
                                                                    Point3DReadOnly lineSegmentStart,
                                                                    Point3DReadOnly lineSegmentEnd)
   {
      if (doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, lineSegmentStart, lineSegmentEnd))
      { // Since an intersection exists, it is now the same as computing the intersection line-plane
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         return intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, lineSegmentStart, lineDirection);
      }
      else
      {
         return null;
      }
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and an axis-aligned bounding
    * box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the ray origin lies on the boundary of the bounding box it is considered as intersection
    * point.
    * <li>if the ray is colinear with a boundary of the bounding box, the points where the ray
    * first/last intersects with the bounding box boundary are returned as intersection points.
    * </ul>
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenRay2DAndBoundingBox2D(Point2DReadOnly boundingBoxMin,
                                                              Point2DReadOnly boundingBoxMax,
                                                              Point2DReadOnly rayOrigin,
                                                              Vector2DReadOnly rayDirection,
                                                              Point2DBasics firstIntersectionToPack,
                                                              Point2DBasics secondIntersectionToPack)
   {
      double firstPointOnLineX = rayOrigin.getX();
      double firstPointOnLineY = rayOrigin.getY();
      double secondPointOnLineX = rayOrigin.getX() + rayDirection.getX();
      double secondPointOnLineY = rayOrigin.getY() + rayDirection.getY();
      return intersectionBetweenLine2DAndBoundingBox2DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           firstPointOnLineX,
                                                           firstPointOnLineY,
                                                           false,
                                                           secondPointOnLineX,
                                                           secondPointOnLineY,
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the intersection between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rayOriginX         the x-coordinate of a point located on the ray. Not modified.
    * @param rayOriginY         the y-coordinate of a point located on the ray. Not modified.
    * @param rayDirectionX      the x-component of the direction of the ray. Not modified.
    * @param rayDirectionY      the y-component of the direction of the ray. Not modified.
    * @param lineSegmentStartX  x-coordinate of the first endpoint of the line segment.
    * @param lineSegmentStartY  y-coordinate of the first endpoint of the line segment.
    * @param lineSegmentEndX    x-coordinate of the second endpoint of the line segment.
    * @param lineSegmentEndY    y-coordinate of the second endpoint of the line segment.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the ray intersects the line segment, {@code false} otherwise.
    */
   public static boolean intersectionBetweenRay2DAndLineSegment2D(double rayOriginX,
                                                                  double rayOriginY,
                                                                  double rayDirectionX,
                                                                  double rayDirectionY,
                                                                  double lineSegmentStartX,
                                                                  double lineSegmentStartY,
                                                                  double lineSegmentEndX,
                                                                  double lineSegmentEndY,
                                                                  Point2DBasics intersectionToPack)
   {
      double start1x = rayOriginX;
      double start1y = rayOriginY;
      double end1x = rayOriginX + rayDirectionX;
      double end1y = rayOriginY + rayDirectionY;
      double start2x = lineSegmentStartX;
      double start2y = lineSegmentStartY;
      double end2x = lineSegmentEndX;
      double end2y = lineSegmentEndY;
      return intersectionBetweenTwoLine2DsImpl(start1x, start1y, false, end1x, end1y, true, start2x, start2y, false, end2x, end2y, false, intersectionToPack);

   }

   /**
    * Computes the intersection between a 2D and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect, this
    * method returns {@code null}.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns that
    * same endpoint.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rayOrigin        a point located on the ray. Not modified.
    * @param rayDirection     the direction of the ray. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return the 2D point of intersection if it exist, {@code null} otherwise.
    */
   public static Point2D intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin,
                                                                  Vector2DReadOnly rayDirection,
                                                                  Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(),
                                                                 rayOrigin.getY(),
                                                                 rayDirection.getX(),
                                                                 rayDirection.getY(),
                                                                 lineSegmentStart.getX(),
                                                                 lineSegmentStart.getY(),
                                                                 lineSegmentEnd.getX(),
                                                                 lineSegmentEnd.getY(),
                                                                 intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   /**
    * Computes the intersection between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rayOrigin          a point located on the ray. Not modified.
    * @param rayDirection       the direction of the ray. Not modified.
    * @param lineSegmentStart   the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd     the second endpoint of the line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the ray intersects the line segment, {@code false} otherwise.
    */
   public static boolean intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin,
                                                                  Vector2DReadOnly rayDirection,
                                                                  Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd,
                                                                  Point2DBasics intersectionToPack)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(),
                                                      rayOrigin.getY(),
                                                      rayDirection.getX(),
                                                      rayDirection.getY(),
                                                      lineSegmentStart.getX(),
                                                      lineSegmentStart.getY(),
                                                      lineSegmentEnd.getX(),
                                                      lineSegmentEnd.getY(),
                                                      intersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and an axis-aligned bounding
    * box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the start of the ray lies on the surface of the bounding box it is considered as
    * intersection point.
    * <li>if the ray is colinear with a surface of the bounding box, the points where the ray
    * first/last intersects with the bounding box (on the bounding box boundary) are returned as
    * intersection points.
    * </ul>
    * </p>
    * 
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException if any of the minimum coordinates of the bounding box is strictly
    *                              greater than the maximum coordinate of the bounding box on the same
    *                              axis.
    */
   public static int intersectionBetweenRay3DAndBoundingBox3D(Point3DReadOnly boundingBoxMin,
                                                              Point3DReadOnly boundingBoxMax,
                                                              Point3DReadOnly rayOrigin,
                                                              Vector3DReadOnly rayDirection,
                                                              Point3DBasics firstIntersectionToPack,
                                                              Point3DBasics secondIntersectionToPack)
   {
      double firstPointOnLineX = rayOrigin.getX();
      double firstPointOnLineY = rayOrigin.getY();
      double firstPointOnLineZ = rayOrigin.getZ();
      double secondPointOnLineX = rayOrigin.getX() + rayDirection.getX();
      double secondPointOnLineY = rayOrigin.getY() + rayDirection.getY();
      double secondPointOnLineZ = rayOrigin.getZ() + rayDirection.getZ();
      return intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMin,
                                                           boundingBoxMax,
                                                           firstPointOnLineX,
                                                           firstPointOnLineY,
                                                           firstPointOnLineZ,
                                                           false,
                                                           secondPointOnLineX,
                                                           secondPointOnLineY,
                                                           secondPointOnLineZ,
                                                           true,
                                                           firstIntersectionToPack,
                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a 3D ray and a 3D box
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boxPosition              the coordinates of the box position. Not modified.
    * @param boxOrientation           the orientation of the box. Not modified.
    * @param boxSize                  the size of the box. Not modified.
    * @param rayOrigin                the origin point of the 3D ray. Not modified.
    * @param rayDirection             the direction of the 3D ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the 3D box. It is either equal to 0, 1,
    *         or 2. If the ray origin is on the surface of the 3D box it is considered an intersection.
    * @throws IllegalArgumentException if {@code boxSize} contains values <= 0.0
    * @see #intersectionBetweenRay3DAndBoundingBox3D(Point3DReadOnly, Point3DReadOnly, Point3DReadOnly,
    *      Vector3DReadOnly, Point3DBasics, Point3DBasics)
    */
   public static int intersectionBetweenRay3DAndBox3D(Point3DReadOnly boxPosition,
                                                      Orientation3DReadOnly boxOrientation,
                                                      Vector3DReadOnly boxSize,
                                                      Point3DReadOnly rayOrigin,

                                                      Vector3DReadOnly rayDirection,
                                                      Point3DBasics firstIntersectionToPack,
                                                      Point3DBasics secondIntersectionToPack)
   {

      if (boxSize.getX() <= 0.0 || boxSize.getY() <= 0.0 || boxSize.getZ() <= 0.0)
         throw new IllegalArgumentException("The box size has to be positive and bigger 0.");

      // if arguments null:
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      if (firstIntersectionToPack == null)
         firstIntersectionToPack = new Point3D();
      if (secondIntersectionToPack == null)
         secondIntersectionToPack = new Point3D();

      firstIntersectionToPack.set(rayOrigin);
      firstIntersectionToPack.sub(boxPosition);
      boxOrientation.inverseTransform(firstIntersectionToPack);

      double firstPointOnLineX = firstIntersectionToPack.getX();
      double firstPointOnLineY = firstIntersectionToPack.getY();
      double firstPointOnLineZ = firstIntersectionToPack.getZ();

      // construct the second point on line
      secondIntersectionToPack.set(rayDirection);
      boxOrientation.inverseTransform(secondIntersectionToPack);

      double secondPointOnLineX = firstPointOnLineX + secondIntersectionToPack.getX();
      double secondPointOnLineY = firstPointOnLineY + secondIntersectionToPack.getY();
      double secondPointOnLineZ = firstPointOnLineZ + secondIntersectionToPack.getZ();

      double boundingBoxMinX = -0.5 * boxSize.getX();
      double boundingBoxMinY = -0.5 * boxSize.getY();
      double boundingBoxMinZ = -0.5 * boxSize.getZ();
      double boundingBoxMaxX = 0.5 * boxSize.getX();
      double boundingBoxMaxY = 0.5 * boxSize.getY();
      double boundingBoxMaxZ = 0.5 * boxSize.getZ();

      int numIntersections = intersectionBetweenLine3DAndBoundingBox3DImpl(boundingBoxMinX,
                                                                           boundingBoxMinY,
                                                                           boundingBoxMinZ,
                                                                           boundingBoxMaxX,
                                                                           boundingBoxMaxY,
                                                                           boundingBoxMaxZ,
                                                                           firstPointOnLineX,
                                                                           firstPointOnLineY,
                                                                           firstPointOnLineZ,
                                                                           false,
                                                                           secondPointOnLineX,
                                                                           secondPointOnLineY,
                                                                           secondPointOnLineZ,
                                                                           true,
                                                                           firstIntersectionToPack,
                                                                           secondIntersectionToPack);

      boxOrientation.transform(firstIntersectionToPack);
      firstIntersectionToPack.add(boxPosition);
      boxOrientation.transform(secondIntersectionToPack);
      secondIntersectionToPack.add(boxPosition);

      return numIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the ray and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws IllegalArgumentException if either {@code cylinderLength < 0} or
    *                                  {@code cylinderRadius < 0}.
    */
   public static int intersectionBetweenRay3DAndCylinder3D(double cylinderLength,
                                                           double cylinderRadius,
                                                           Point3DReadOnly cylinderPosition,
                                                           Vector3DReadOnly cylinderAxis,
                                                           Point3DReadOnly rayOrigin,
                                                           Vector3DReadOnly rayDirection,
                                                           Point3DBasics firstIntersectionToPack,
                                                           Point3DBasics secondIntersectionToPack)
   {
      double cylinderPositionX = cylinderPosition.getX();
      double cylinderPositionY = cylinderPosition.getY();
      double cylinderPositionZ = cylinderPosition.getZ();
      double cylinderAxisX = cylinderAxis.getX();
      double cylinderAxisY = cylinderAxis.getY();
      double cylinderAxisZ = cylinderAxis.getZ();
      double startX = rayOrigin.getX();
      double startY = rayOrigin.getY();
      double startZ = rayOrigin.getZ();
      double endX = rayOrigin.getX() + rayDirection.getX();
      double endY = rayOrigin.getY() + rayDirection.getY();
      double endZ = rayOrigin.getZ() + rayDirection.getZ();
      return intersectionBetweenLine3DAndCylinder3DImpl(cylinderLength,
                                                        cylinderRadius,
                                                        cylinderPositionX,
                                                        cylinderPositionY,
                                                        cylinderPositionZ,
                                                        cylinderAxisX,
                                                        cylinderAxisY,
                                                        cylinderAxisZ,
                                                        cylinderAxis instanceof UnitVector3DReadOnly,
                                                        startX,
                                                        startY,
                                                        startZ,
                                                        false,
                                                        endX,
                                                        endY,
                                                        endZ,
                                                        true,
                                                        firstIntersectionToPack,
                                                        secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and a ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the ray and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException if either {@code radiusX}, {@code radiusY}, or {@code radiusZ}
    *                                  is negative.
    */
   public static int intersectionBetweenRay3DAndEllipsoid3D(double radiusX,
                                                            double radiusY,
                                                            double radiusZ,
                                                            Point3DReadOnly rayOrigin,
                                                            Vector3DReadOnly rayDirection,
                                                            Point3DBasics firstIntersectionToPack,
                                                            Point3DBasics secondIntersectionToPack)
   {
      double startX = rayOrigin.getX();
      double startY = rayOrigin.getY();
      double startZ = rayOrigin.getZ();
      double endX = rayOrigin.getX() + rayDirection.getX();
      double endY = rayOrigin.getY() + rayDirection.getY();
      double endZ = rayOrigin.getZ() + rayDirection.getZ();
      return intersectionBetweenLine3DAndEllipsoid3DImpl(radiusX,
                                                         radiusY,
                                                         radiusZ,
                                                         startX,
                                                         startY,
                                                         startZ,
                                                         false,
                                                         endX,
                                                         endY,
                                                         endZ,
                                                         true,
                                                         firstIntersectionToPack,
                                                         secondIntersectionToPack);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1x      x-coordinate of a point located on the first line.
    * @param pointOnLine1y      y-coordinate of a point located on the first line.
    * @param lineDirection1x    x-component of the first line direction.
    * @param lineDirection1y    y-component of the first line direction.
    * @param pointOnLine2x      x-coordinate of a point located on the second line.
    * @param pointOnLine2y      y-coordinate of a point located on the second line.
    * @param lineDirection2x    x-component of the second line direction.
    * @param lineDirection2y    y-component of the second line direction.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    */
   public static boolean intersectionBetweenTwoLine2Ds(double pointOnLine1x,
                                                       double pointOnLine1y,
                                                       double lineDirection1x,
                                                       double lineDirection1y,
                                                       double pointOnLine2x,
                                                       double pointOnLine2y,
                                                       double lineDirection2x,
                                                       double lineDirection2y,
                                                       Point2DBasics intersectionToPack)
   {
      double alpha = percentageOfIntersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                               pointOnLine1y,
                                                               lineDirection1x,
                                                               lineDirection1y,
                                                               pointOnLine2x,
                                                               pointOnLine2y,
                                                               lineDirection2x,
                                                               lineDirection2y);
      if (Double.isInfinite(alpha))
      { // The lines are collinear, let's set the intersection to pointOnLine1
         if (intersectionToPack != null)
            intersectionToPack.set(pointOnLine1x, pointOnLine1y);
         return true;
      }
      if (Double.isNaN(alpha))
      {
         if (intersectionToPack != null)
            intersectionToPack.setToNaN();
         return false;
      }

      if (intersectionToPack != null)
      {
         intersectionToPack.set(pointOnLine1x + alpha * lineDirection1x, pointOnLine1y + alpha * lineDirection1y);
      }
      return true;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @return the 2D point of intersection if the two lines intersect, {@code null} otherwise.
    */
   public static Point2D intersectionBetweenTwoLine2Ds(Point2DReadOnly firstPointOnLine1,
                                                       Point2DReadOnly secondPointOnLine1,
                                                       Point2DReadOnly firstPointOnLine2,
                                                       Point2DReadOnly secondPointOnLine2)
   {
      Point2D intersection = new Point2D();

      boolean success = intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, intersection);

      if (!success)
         return null;
      else
         return intersection;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    */
   public static boolean intersectionBetweenTwoLine2Ds(Point2DReadOnly firstPointOnLine1,
                                                       Point2DReadOnly secondPointOnLine1,
                                                       Point2DReadOnly firstPointOnLine2,
                                                       Point2DReadOnly secondPointOnLine2,
                                                       Point2DBasics intersectionToPack)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();

      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                               pointOnLine1y,
                                                               lineDirection1x,
                                                               lineDirection1y,
                                                               pointOnLine2x,
                                                               pointOnLine2y,
                                                               lineDirection2x,
                                                               lineDirection2y,
                                                               intersectionToPack);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnLine1   point located on the first line. Not modified.
    * @param lineDirection1 the first line direction. Not modified.
    * @param pointOnLine2   point located on the second line. Not modified.
    * @param lineDirection2 the second line direction. Not modified.
    * @return the 2D point of intersection if the two lines intersect, {@code null} otherwise.
    */
   public static Point2D intersectionBetweenTwoLine2Ds(Point2DReadOnly pointOnLine1,
                                                       Vector2DReadOnly lineDirection1,
                                                       Point2DReadOnly pointOnLine2,
                                                       Vector2DReadOnly lineDirection2)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1       point located on the first line. Not modified.
    * @param lineDirection1     the first line direction. Not modified.
    * @param pointOnLine2       point located on the second line. Not modified.
    * @param lineDirection2     the second line direction. Not modified.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    */
   public static boolean intersectionBetweenTwoLine2Ds(Point2DReadOnly pointOnLine1,
                                                       Vector2DReadOnly lineDirection1,
                                                       Point2DReadOnly pointOnLine2,
                                                       Vector2DReadOnly lineDirection2,
                                                       Point2DBasics intersectionToPack)
   {
      return intersectionBetweenTwoLine2Ds(pointOnLine1.getX(),
                                           pointOnLine1.getY(),
                                           lineDirection1.getX(),
                                           lineDirection1.getY(),
                                           pointOnLine2.getX(),
                                           pointOnLine2.getY(),
                                           lineDirection2.getX(),
                                           lineDirection2.getY(),
                                           intersectionToPack);
   }

   /**
    * Computes the intersection between two 2D line segments each defined by their two 2D endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns {@code true}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    *
    * @param lineSegmentStart1x x-coordinate of the first endpoint of the first line segment.
    * @param lineSegmentStart1y y-coordinate of the first endpoint of the first line segment.
    * @param lineSegmentEnd1x   x-coordinate of the second endpoint of the first line segment.
    * @param lineSegmentEnd1y   y-coordinate of the second endpoint of the first line segment.
    * @param lineSegmentStart2x x-coordinate of the first endpoint of the second line segment.
    * @param lineSegmentStart2y y-coordinate of the first endpoint of the second line segment.
    * @param lineSegmentEnd2x   x-coordinate of the second endpoint of the second line segment.
    * @param lineSegmentEnd2y   y-coordinate of the second endpoint of the second line segment.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    */
   public static boolean intersectionBetweenTwoLineSegment2Ds(double lineSegmentStart1x,
                                                              double lineSegmentStart1y,
                                                              double lineSegmentEnd1x,
                                                              double lineSegmentEnd1y,
                                                              double lineSegmentStart2x,
                                                              double lineSegmentStart2y,
                                                              double lineSegmentEnd2x,
                                                              double lineSegmentEnd2y,
                                                              Point2DBasics intersectionToPack)
   {
      double start1x = lineSegmentStart1x;
      double start1y = lineSegmentStart1y;
      double end1x = lineSegmentEnd1x;
      double end1y = lineSegmentEnd1y;
      double start2x = lineSegmentStart2x;
      double start2y = lineSegmentStart2y;
      double end2x = lineSegmentEnd2x;
      double end2y = lineSegmentEnd2y;
      return intersectionBetweenTwoLine2DsImpl(start1x, start1y, false, end1x, end1y, false, start2x, start2y, false, end2x, end2y, false, intersectionToPack);
   }

   /**
    * Computes the intersection between two 2D line segments each defined by their two 2D endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect, this method returns {@code null}.
    * <li>When the two line segments are collinear, if the two line segments do not overlap do not have
    * at least one common endpoint, this method returns {@code null}.
    * <li>When the two line segments have a common endpoint, this method returns the common endpoint as
    * the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the intersection point if it exists, {@code null} otherwise.
    */
   public static Point2D intersectionBetweenTwoLineSegment2Ds(Point2DReadOnly lineSegmentStart1,
                                                              Point2DReadOnly lineSegmentEnd1,
                                                              Point2DReadOnly lineSegmentStart2,
                                                              Point2DReadOnly lineSegmentEnd2)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, intersection);
      if (!success)
         return null;
      else
         return intersection;
   }

   /**
    * Computes the intersection between two 2D line segments each defined by their two 2D endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns {@code true}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart1  the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1    the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2  the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2    the second endpoint of the second line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    */
   public static boolean intersectionBetweenTwoLineSegment2Ds(Point2DReadOnly lineSegmentStart1,
                                                              Point2DReadOnly lineSegmentEnd1,
                                                              Point2DReadOnly lineSegmentStart2,
                                                              Point2DReadOnly lineSegmentEnd2,
                                                              Point2DBasics intersectionToPack)
   {
      return intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1.getX(),
                                                  lineSegmentStart1.getY(),
                                                  lineSegmentEnd1.getX(),
                                                  lineSegmentEnd1.getY(),
                                                  lineSegmentStart2.getX(),
                                                  lineSegmentStart2.getY(),
                                                  lineSegmentEnd2.getX(),
                                                  lineSegmentEnd2.getY(),
                                                  intersectionToPack);
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below {@link #ONE_TRILLIONTH}, this methods
    * fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@code angleThreshold}, this methods fails and
    * returns {@code false}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code pointOnIntersectionToPack} and {@code intersectionDirectionToPack} are set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param angleThreshold              the minimum angle between the two planes required to do the
    *                                    calculation.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleThreshold</tt> &notin; [0; <i>pi</i>/2]
    */
   public static boolean intersectionBetweenTwoPlane3Ds(Point3DReadOnly pointOnPlane1,
                                                        Vector3DReadOnly planeNormal1,
                                                        Point3DReadOnly pointOnPlane2,
                                                        Vector3DReadOnly planeNormal2,
                                                        double angleThreshold,
                                                        Point3DBasics pointOnIntersectionToPack,
                                                        Vector3DBasics intersectionDirectionToPack)
   {
      if (angleThreshold < 0.0 || angleThreshold > HALF_PI)
         throw new IllegalArgumentException("The angle threshold has to be inside the interval: [0.0 ; Math.PI / 2.0]");

      pointOnIntersectionToPack.setToNaN();
      intersectionDirectionToPack.setToNaN();

      double normalMagnitude1 = planeNormal1.norm();

      if (normalMagnitude1 < ONE_TRILLIONTH)
         return false;

      double normalMagnitude2 = planeNormal2.norm();

      if (normalMagnitude2 < ONE_TRILLIONTH)
         return false;

      // Check if planes are parallel.
      if (Math.abs(planeNormal1.dot(planeNormal2) / (normalMagnitude1 * normalMagnitude2)) > EuclidCoreTools.cos(Math.abs(angleThreshold)))
         return false;

      intersectionDirectionToPack.cross(planeNormal1, planeNormal2);
      double det = intersectionDirectionToPack.normSquared();

      // d1 = planeNormal1 . pointOnPlane1
      double d1 = planeNormal1.getX() * pointOnPlane1.getX() + planeNormal1.getY() * pointOnPlane1.getY() + planeNormal1.getZ() * pointOnPlane1.getZ();
      // d2 = planeNormal2 . pointOnPlane2
      double d2 = planeNormal2.getX() * pointOnPlane2.getX() + planeNormal2.getY() * pointOnPlane2.getY() + planeNormal2.getZ() * pointOnPlane2.getZ();

      // normal3Cross2 = intersectionDirectionToPack x planeNormal2
      double normal3Cross2X = intersectionDirectionToPack.getY() * planeNormal2.getZ() - intersectionDirectionToPack.getZ() * planeNormal2.getY();
      double normal3Cross2Y = intersectionDirectionToPack.getZ() * planeNormal2.getX() - intersectionDirectionToPack.getX() * planeNormal2.getZ();
      double normal3Cross2Z = intersectionDirectionToPack.getX() * planeNormal2.getY() - intersectionDirectionToPack.getY() * planeNormal2.getX();

      // normal1Cross3 = planeNormal1 x intersectionDirectionToPack
      double normal1Cross3X = planeNormal1.getY() * intersectionDirectionToPack.getZ() - planeNormal1.getZ() * intersectionDirectionToPack.getY();
      double normal1Cross3Y = planeNormal1.getZ() * intersectionDirectionToPack.getX() - planeNormal1.getX() * intersectionDirectionToPack.getZ();
      double normal1Cross3Z = planeNormal1.getX() * intersectionDirectionToPack.getY() - planeNormal1.getY() * intersectionDirectionToPack.getX();

      // normal2Cross1 = planeNormal2 x planeNormal1
      double normal2Cross1X = -intersectionDirectionToPack.getX();
      double normal2Cross1Y = -intersectionDirectionToPack.getY();
      double normal2Cross1Z = -intersectionDirectionToPack.getZ();

      intersectionDirectionToPack.scale(1.0 / EuclidCoreTools.squareRoot(det));

      double normal3DotPoint1 = intersectionDirectionToPack.getX() * pointOnPlane1.getX() + intersectionDirectionToPack.getY() * pointOnPlane1.getY()
                                + intersectionDirectionToPack.getZ() * pointOnPlane1.getZ();
      double normal3DotPoint2 = intersectionDirectionToPack.getX() * pointOnPlane2.getX() + intersectionDirectionToPack.getY() * pointOnPlane2.getY()
                                + intersectionDirectionToPack.getZ() * pointOnPlane2.getZ();
      double d3 = 0.5 * (normal3DotPoint1 + normal3DotPoint2);

      pointOnIntersectionToPack.set(d1 * normal3Cross2X + d2 * normal1Cross3X + d3 * normal2Cross1X,
                                    d1 * normal3Cross2Y + d2 * normal1Cross3Y + d3 * normal2Cross1Y,
                                    d1 * normal3Cross2Z + d2 * normal1Cross3Z + d3 * normal2Cross1Z);
      pointOnIntersectionToPack.scale(-1.0 / det);

      return true;
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below {@link #ONE_TRILLIONTH}, this methods
    * fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@link #ONE_MILLIONTH}, this methods fails and
    * returns {@code false}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code pointOnIntersectionToPack} and {@code intersectionDirectionToPack} are set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    */
   public static boolean intersectionBetweenTwoPlane3Ds(Point3DReadOnly pointOnPlane1,
                                                        Vector3DReadOnly planeNormal1,
                                                        Point3DReadOnly pointOnPlane2,
                                                        Vector3DReadOnly planeNormal2,
                                                        Point3DBasics pointOnIntersectionToPack,
                                                        Vector3DBasics intersectionDirectionToPack)
   {
      return intersectionBetweenTwoPlane3Ds(pointOnPlane1,
                                            planeNormal1,
                                            pointOnPlane2,
                                            planeNormal2,
                                            ONE_MILLIONTH,
                                            pointOnIntersectionToPack,
                                            intersectionDirectionToPack);
   }

   /**
    * This methods verifies that the given set of three lengths represents a triangle. A valid triangle
    * with three edges A, B, and C verifies the three following inequalities:
    * <ul>
    * <li>|A| + |B| > |C|
    * <li>|B| + |C| > |A|
    * <li>|C| + |A| > |B|
    * </ul>
    * <a href="https://opencurriculum.org/5534/the-triangle-inequality/"> Useful link</a>.
    * </p>
    *
    * @param lengthSideA the length of the side A.
    * @param lengthSideB the length of the side B.
    * @param lengthSideC the length of the side C.
    * @return {@code true} if the lengths represents the three sides of a triangle, {@code false}
    *         otherwise.
    * @throws IllegalArgumentException if any of the three lengths is negative.
    */
   public static boolean isFormingTriangle(double lengthSideA, double lengthSideB, double lengthSideC)
   {
      if (lengthSideA < 0.0)
         throw new IllegalArgumentException("The side A cannot have a negative length, lengthSideA = " + lengthSideA);
      if (lengthSideB < 0.0)
         throw new IllegalArgumentException("The side B cannot have a negative length, lengthSideB = " + lengthSideB);
      if (lengthSideC < 0.0)
         throw new IllegalArgumentException("The side C cannot have a negative length, lengthSideC = " + lengthSideC);

      if (lengthSideA + lengthSideB <= lengthSideC)
         return false;
      if (lengthSideB + lengthSideC <= lengthSideA)
         return false;
      if (lengthSideA + lengthSideC <= lengthSideB)
         return false;
      return true;
   }

   /**
    * Determines if the query is: ahead of the ray, i.e. the projection onto ray lies in front of the
    * ray's origin, behind the ray, or neither, i.e. the projection is equal to the ray origin.
    *
    * @param pointX        the x-coordinate of the query.
    * @param pointY        the y-coordinate of the query.
    * @param rayOriginX    the x-coordinate of the ray's origin.
    * @param rayOriginY    the y-coordinate of the ray's origin.
    * @param rayDirectionX the x-component of the ray's direction.
    * @param rayDirectionY the y-component of the ray's direction.
    * @return {@link Location#AHEAD} if the query is located in front of the ray,
    *         {@link Location#BEHIND} if the query is behind the ray, and {@code null} if the query's
    *         projection onto the ray is exactly equal to the ray origin.
    */
   public static Location whichPartOfRay2DIsPoint2DOn(double pointX,
                                                      double pointY,
                                                      double rayOriginX,
                                                      double rayOriginY,
                                                      double rayDirectionX,
                                                      double rayDirectionY)
   {
      double rayStartToVertexX = pointX - rayOriginX;
      double rayStartToVertexY = pointY - rayOriginY;
      double dotProduct = rayStartToVertexX * rayDirectionX + rayStartToVertexY * rayDirectionY;
      if (dotProduct > 0.0)
         return Location.AHEAD;
      else if (dotProduct < 0.0)
         return Location.BEHIND;
      else
         return null;
   }

   /**
    * Determines if the query is: ahead of the ray, i.e. the projection onto ray lies in front of the
    * ray's origin, behind the ray, or neither, i.e. the projection is equal to the ray origin.
    *
    * @param point        the query. Not modified.
    * @param rayOrigin    the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @return {@link Location#AHEAD} if the query is located in front of the ray,
    *         {@link Location#BEHIND} if the query is behind the ray, and {@code null} if the query's
    *         projection onto the ray is exactly equal to the ray origin.
    */
   public static Location whichPartOfRay2DIsPoint2DOn(Point2DReadOnly point, Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection)
   {
      return whichPartOfRay2DIsPoint2DOn(point.getX(), point.getY(), rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY());
   }

   /**
    * Determines if the query is: ahead of the ray, i.e. the projection onto ray lies in front of the
    * ray's origin, behind the ray, or neither, i.e. the projection is equal to the ray origin.
    *
    * @param pointX        the x-coordinate of the query.
    * @param pointY        the y-coordinate of the query.
    * @param rayOriginX    the x-coordinate of the ray's origin.
    * @param rayOriginY    the y-coordinate of the ray's origin.
    * @param rayDirectionX the x-component of the ray's direction.
    * @param rayDirectionY the y-component of the ray's direction.
    * @return {@code true} if the query is located in front of the ray.
    */
   public static boolean isPoint2DInFrontOfRay2D(double pointX, double pointY, double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY)
   {
      return whichPartOfRay2DIsPoint2DOn(pointX, pointY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY) != Location.BEHIND;
   }

   /**
    * Determines if the query is: ahead of the ray, i.e. the projection onto ray lies in front of the
    * ray's origin, behind the ray, or neither, i.e. the projection is equal to the ray origin.
    *
    * @param point        the query. Not modified.
    * @param rayOrigin    the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @return {@code true} if the query is located in front of the ray.
    */
   public static boolean isPoint2DInFrontOfRay2D(Point2DReadOnly point, Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection)
   {
      return whichPartOfRay2DIsPoint2DOn(point, rayOrigin, rayDirection) != Location.BEHIND;
   }

   /**
    * Returns {@code true} only if the point is inside the triangle defined by the vertices a, b, and
    * c. The triangle can be clockwise or counter-clockwise ordered.
    *
    * @param point the point to check if lying inside the triangle. Not modified.
    * @param a     first vertex of the triangle. Not modified.
    * @param b     second vertex of the triangle. Not modified.
    * @param c     third vertex of the triangle. Not modified.
    * @return {@code true} if the query is exactly inside the triangle. {@code false} if the query
    *         point is outside triangle or exactly on an edge of the triangle.
    */
   public static boolean isPoint2DInsideTriangleABC(Point2DReadOnly point, Point2DReadOnly a, Point2DReadOnly b, Point2DReadOnly c)
   {
      // This makes the assertion working for both clockwise and counter-clockwise ordered vertices.
      boolean checkForLeftSide = isPoint2DOnLeftSideOfLine2D(b, a, c);

      if (isPoint2DOnSideOfLine2D(point, a, b, checkForLeftSide))
         return false;
      if (isPoint2DOnSideOfLine2D(point, b, c, checkForLeftSide))
         return false;
      if (isPoint2DOnSideOfLine2D(point, c, a, checkForLeftSide))
         return false;

      return true;
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link #IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param pointX         the x-coordinate of the query.
    * @param pointY         the y-coordinate of the query.
    * @param pointOnLineX   the x-coordinate of a point located on the line.
    * @param pointOnLineY   the y-coordinate of a point located on the line.
    * @param lineDirectionX the x-component of the direction of the line.
    * @param lineDirectionY the y-component of the direction of the line.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    */
   public static boolean isPoint2DOnLine2D(double pointX, double pointY, double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      return distanceFromPoint2DToLine2D(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY) < IS_POINT_ON_LINE_EPS;
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link #IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param pointX        the x-coordinate of the query.
    * @param pointY        the y-coordinate of the query.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    */
   public static boolean isPoint2DOnLine2D(double pointX, double pointY, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return distanceFromPoint2DToLine2D(pointX, pointY, pointOnLine, lineDirection) < IS_POINT_ON_LINE_EPS;
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link #IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    */
   public static boolean isPoint2DOnLine2D(Point2DReadOnly point, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return isPoint2DOnLine2D(point.getX(), point.getY(), pointOnLine, lineDirection);
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link #IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param point            the coordinates of the query. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    */
   public static boolean isPoint2DOnLineSegment2D(Point2DReadOnly point, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return distanceFromPoint2DToLineSegment2D(point, lineSegmentStart, lineSegmentEnd) < IS_POINT_ON_LINE_EPS;
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line defined by two
    * points. The idea of "side" is determined based on order of {@code firstPointOnLine} and
    * {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param pointX            the x-coordinate of the query point.
    * @param pointY            the y-coordinate of the query point.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(double pointX, double pointY, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      return whichSideOfLine2DIsPoint2DOn(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line. The idea of
    * "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(double pointX, double pointY, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      double pointOnLineX = pointOnLine.getX();
      double pointOnLineY = pointOnLine.getY();
      double lineDirectionX = lineDirection.getX();
      double lineDirectionY = lineDirection.getY();
      return whichSideOfLine2DIsPoint2DOn(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line defined by two
    * points. The idea of "side" is determined based on order of {@code firstPointOnLine} and
    * {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(Point2DReadOnly point, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      return whichSideOfLine2DIsPoint2DOn(point.getX(), point.getY(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line. The idea of
    * "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param point         the query point. Not modified.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(Point2DReadOnly point, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return whichSideOfLine2DIsPoint2DOn(point.getX(), point.getY(), pointOnLine, lineDirection);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line. The idea of
    * "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param pointX         the x-coordinate of the query point.
    * @param pointY         the y-coordinate of the query point.
    * @param pointOnLineX   the x-coordinate of a point positioned on the infinite line.
    * @param pointOnLineY   the y-coordinate of a point positioned on the infinite line.
    * @param lineDirectionX the x-component of the direction of the infinite line.
    * @param lineDirectionY the y-component of the direction of the infinite line.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(double pointX,
                                                       double pointY,
                                                       double pointOnLineX,
                                                       double pointOnLineY,
                                                       double lineDirectionX,
                                                       double lineDirectionY)
   {
      double dx = pointX - pointOnLineX;
      double dy = pointY - pointOnLineY;
      double crossProduct = lineDirectionX * dy - dx * lineDirectionY;
      if (crossProduct > 0.0)
         return Location.LEFT;
      else if (crossProduct < 0.0)
         return Location.RIGHT;
      else
         return null;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left side of an infinitely long
    * line defined by two points. "Left side" is determined based on order of {@code firstPointOnLine}
    * and {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on the left side of this line
    * has a negative x coordinate.
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@code true} if the point is on the left side of the line, {@code false} if the point is
    *         on the right side or exactly on the line.
    */
   public static boolean isPoint2DOnLeftSideOfLine2D(Point2DReadOnly point, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      return whichSideOfLine2DIsPoint2DOn(point, firstPointOnLine, secondPointOnLine) == Location.LEFT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the right side of an infinitely long
    * line defined by two points. "Right side" is determined based on order of {@code firstPointOnLine}
    * and {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on the right side of this
    * line has a positive x coordinate.
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@code true} if the point is on the right side of the line, {@code false} if the point is
    *         on the left side or exactly on the line.
    */
   public static boolean isPoint2DOnRightSideOfLine2D(Point2DReadOnly point, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      return whichSideOfLine2DIsPoint2DOn(point, firstPointOnLine, secondPointOnLine) == Location.RIGHT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line. The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param pointX         the x-coordinate of the query point.
    * @param pointY         the y-coordinate of the query point.
    * @param pointOnLineX   the x-coordinate of a point positioned on the infinite line.
    * @param pointOnLineY   the y-coordinate of a point positioned on the infinite line.
    * @param lineDirectionX the x-component of the direction of the infinite line.
    * @param lineDirectionY the y-component of the direction of the infinite line.
    * @param testLeftSide   the query of the side, when equal to {@code true} this will test for the
    *                       left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(double, double, double, double, double, double)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(double pointX,
                                                 double pointY,
                                                 double pointOnLineX,
                                                 double pointOnLineY,
                                                 double lineDirectionX,
                                                 double lineDirectionY,
                                                 boolean testLeftSide)
   {
      Location side = whichSideOfLine2DIsPoint2DOn(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
      return testLeftSide ? side == Location.LEFT : side == Location.RIGHT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line defined by two points. The idea of "side" is determined based on order of
    * {@code firstPointOnLine} and {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param pointX            the x-coordinate of the query point.
    * @param pointY            the y-coordinate of the query point.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param testLeftSide      the query of the side, when equal to {@code true} this will test for the
    *                          left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(double, double, Point2DReadOnly, Point2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(double pointX,
                                                 double pointY,
                                                 Point2DReadOnly firstPointOnLine,
                                                 Point2DReadOnly secondPointOnLine,
                                                 boolean testLeftSide)
   {
      Location side = whichSideOfLine2DIsPoint2DOn(pointX, pointY, firstPointOnLine, secondPointOnLine);
      return testLeftSide ? side == Location.LEFT : side == Location.RIGHT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line. The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @param testLeftSide  the query of the side, when equal to {@code true} this will test for the
    *                      left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(double, double, Point2DReadOnly, Vector2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(double pointX,
                                                 double pointY,
                                                 Point2DReadOnly pointOnLine,
                                                 Vector2DReadOnly lineDirection,
                                                 boolean testLeftSide)
   {
      Location side = whichSideOfLine2DIsPoint2DOn(pointX, pointY, pointOnLine, lineDirection);
      return testLeftSide ? side == Location.LEFT : side == Location.RIGHT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line defined by two points. The idea of "side" is determined based on order of
    * {@code firstPointOnLine} and {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param testLeftSide      the query of the side, when equal to {@code true} this will test for the
    *                          left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(Point2DReadOnly, Point2DReadOnly, Point2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(Point2DReadOnly point,
                                                 Point2DReadOnly firstPointOnLine,
                                                 Point2DReadOnly secondPointOnLine,
                                                 boolean testLeftSide)
   {
      Location side = whichSideOfLine2DIsPoint2DOn(point, firstPointOnLine, secondPointOnLine);
      return testLeftSide ? side == Location.LEFT : side == Location.RIGHT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line. The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point         the query point. Not modified.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @param testLeftSide  the query of the side, when equal to {@code true} this will test for the
    *                      left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(Point2DReadOnly, Point2DReadOnly, Vector2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(Point2DReadOnly point, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, boolean testLeftSide)
   {
      Location side = whichSideOfLine2DIsPoint2DOn(point, pointOnLine, lineDirection);
      return testLeftSide ? side == Location.LEFT : side == Location.RIGHT;
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the plane.
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointZ        the z-coordinate of the query point.
    * @param pointOnPlaneX the x-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneY the y-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneZ the z-coordinate of a point positioned on the infinite plane.
    * @param planeNormalX  the x-component of the normal of the infinite plane.
    * @param planeNormalY  the y-component of the normal of the infinite plane.
    * @param planeNormalZ  the z-component of the normal of the infinite plane.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(double pointX,
                                                        double pointY,
                                                        double pointZ,
                                                        double pointOnPlaneX,
                                                        double pointOnPlaneY,
                                                        double pointOnPlaneZ,
                                                        double planeNormalX,
                                                        double planeNormalY,
                                                        double planeNormalZ)
   {
      double dx = (pointX - pointOnPlaneX) * planeNormalX;
      double dy = (pointY - pointOnPlaneY) * planeNormalY;
      double dz = (pointZ - pointOnPlaneZ) * planeNormalZ;
      double signedDistance = dx + dy + dz;

      if (signedDistance > 0.0)
         return Location.ABOVE;
      else if (signedDistance < 0.0)
         return Location.BELOW;
      else
         return null;
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(double pointX, double pointY, double pointZ, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX,
                                           pointY,
                                           pointZ,
                                           pointOnPlane.getX(),
                                           pointOnPlane.getY(),
                                           pointOnPlane.getZ(),
                                           planeNormal.getX(),
                                           planeNormal.getY(),
                                           planeNormal.getZ());
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(Point3DReadOnly point, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(point.getX(), point.getY(), point.getZ(), pointOnPlane, planeNormal);
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX              the x-coordinate of the query point.
    * @param pointY              the y-coordinate of the query point.
    * @param pointZ              the z-coordinate of the query point.
    * @param pointOnPlaneX       the x-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneY       the y-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneZ       the z-coordinate of a point positioned on the infinite plane.
    * @param planeFirstTangentX  the x-component of a first tangent of the infinite plane.
    * @param planeFirstTangentY  the y-component of a first tangent of the infinite plane.
    * @param planeFirstTangentZ  the z-component of a first tangent of the infinite plane.
    * @param planeSecondTangentX the x-component of a second tangent of the infinite plane.
    * @param planeSecondTangentY the y-component of a second tangent of the infinite plane.
    * @param planeSecondTangentZ the z-component of a second tangent of the infinite plane.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @see #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double,
    *      double, double)
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(double pointX,
                                                        double pointY,
                                                        double pointZ,
                                                        double pointOnPlaneX,
                                                        double pointOnPlaneY,
                                                        double pointOnPlaneZ,
                                                        double planeFirstTangentX,
                                                        double planeFirstTangentY,
                                                        double planeFirstTangentZ,
                                                        double planeSecondTangentX,
                                                        double planeSecondTangentY,
                                                        double planeSecondTangentZ)
   {
      double planeNormalX = planeFirstTangentY * planeSecondTangentZ - planeFirstTangentZ * planeSecondTangentY;
      double planeNormalY = planeFirstTangentZ * planeSecondTangentX - planeFirstTangentX * planeSecondTangentZ;
      double planeNormalZ = planeFirstTangentX * planeSecondTangentY - planeFirstTangentY * planeSecondTangentX;
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlaneX, pointOnPlaneY, pointOnPlaneZ, planeNormalX, planeNormalY, planeNormalZ);
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @see #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double,
    *      double, double)
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(double pointX,
                                                        double pointY,
                                                        double pointZ,
                                                        Point3DReadOnly pointOnPlane,
                                                        Vector3DReadOnly planeFirstTangent,
                                                        Vector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX,
                                           pointY,
                                           pointZ,
                                           pointOnPlane.getX(),
                                           pointOnPlane.getY(),
                                           pointOnPlane.getZ(),
                                           planeFirstTangent.getX(),
                                           planeFirstTangent.getY(),
                                           planeFirstTangent.getZ(),
                                           planeSecondTangent.getX(),
                                           planeSecondTangent.getY(),
                                           planeSecondTangent.getZ());
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point. Not modified.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @see #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double,
    *      double, double)
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(Point3DReadOnly point,
                                                        Point3DReadOnly pointOnPlane,
                                                        Vector3DReadOnly planeFirstTangent,
                                                        Vector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(point.getX(), point.getY(), point.getZ(), pointOnPlane, planeFirstTangent, planeSecondTangent);
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointZ        the z-coordinate of the query point.
    * @param pointOnPlaneX the x-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneY the y-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneZ the z-coordinate of a point positioned on the infinite plane.
    * @param planeNormalX  the x-component of the normal of the infinite plane.
    * @param planeNormalY  the y-component of the normal of the infinite plane.
    * @param planeNormalZ  the z-component of the normal of the infinite plane.
    * @param testForAbove  the query of the side, when equal to {@code true} this will test for the
    *                      above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(double pointX,
                                                      double pointY,
                                                      double pointZ,
                                                      double pointOnPlaneX,
                                                      double pointOnPlaneY,
                                                      double pointOnPlaneZ,
                                                      double planeNormalX,
                                                      double planeNormalY,
                                                      double planeNormalZ,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(pointX,
                                                    pointY,
                                                    pointZ,
                                                    pointOnPlaneX,
                                                    pointOnPlaneY,
                                                    pointOnPlaneZ,
                                                    planeNormalX,
                                                    planeNormalY,
                                                    planeNormalZ);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @param testForAbove the query of the side, when equal to {@code true} this will test for the
    *                     above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, Point3DReadOnly, Vector3DReadOnly, boolean)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(double pointX,
                                                      double pointY,
                                                      double pointZ,
                                                      Point3DReadOnly pointOnPlane,
                                                      Vector3DReadOnly planeNormal,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @param testForAbove the query of the side, when equal to {@code true} this will test for the
    *                     above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(Point3DReadOnly, Point3DReadOnly, Vector3DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(Point3DReadOnly point, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal, boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    */
   public static boolean isPoint3DAbovePlane3D(double pointX, double pointY, double pointZ, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    */
   public static boolean isPoint3DAbovePlane3D(Point3DReadOnly point, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    */
   public static boolean isPoint3DBelowPlane3D(double pointX, double pointY, double pointZ, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal) == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    */
   public static boolean isPoint3DBelowPlane3D(Point3DReadOnly point, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal) == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX              the x-coordinate of the query point.
    * @param pointY              the y-coordinate of the query point.
    * @param pointZ              the z-coordinate of the query point.
    * @param pointOnPlaneX       the x-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneY       the y-coordinate of a point positioned on the infinite plane.
    * @param pointOnPlaneZ       the z-coordinate of a point positioned on the infinite plane.
    * @param planeFirstTangentX  the x-component of a first tangent of the infinite plane.
    * @param planeFirstTangentY  the y-component of a first tangent of the infinite plane.
    * @param planeFirstTangentZ  the z-component of a first tangent of the infinite plane.
    * @param planeSecondTangentX the x-component of a second tangent of the infinite plane.
    * @param planeSecondTangentY the y-component of a second tangent of the infinite plane.
    * @param planeSecondTangentZ the z-component of a second tangent of the infinite plane.
    * @param testForAbove        the query of the side, when equal to {@code true} this will test for
    *                            the above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double, double, double, double)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(double pointX,
                                                      double pointY,
                                                      double pointZ,
                                                      double pointOnPlaneX,
                                                      double pointOnPlaneY,
                                                      double pointOnPlaneZ,
                                                      double planeFirstTangentX,
                                                      double planeFirstTangentY,
                                                      double planeFirstTangentZ,
                                                      double planeSecondTangentX,
                                                      double planeSecondTangentY,
                                                      double planeSecondTangentZ,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(pointX,
                                                    pointY,
                                                    pointZ,
                                                    pointOnPlaneX,
                                                    pointOnPlaneY,
                                                    pointOnPlaneZ,
                                                    planeFirstTangentX,
                                                    planeFirstTangentY,
                                                    planeFirstTangentZ,
                                                    planeSecondTangentX,
                                                    planeSecondTangentY,
                                                    planeSecondTangentZ);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;

   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @param testForAbove       the query of the side, when equal to {@code true} this will test for
    *                           the above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, Point3DReadOnly, Vector3DReadOnly, Vector3DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(double pointX,
                                                      double pointY,
                                                      double pointZ,
                                                      Point3DReadOnly pointOnPlane,
                                                      Vector3DReadOnly planeFirstTangent,
                                                      Vector3DReadOnly planeSecondTangent,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;

   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point. Not modified.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @param testForAbove       the query of the side, when equal to {@code true} this will test for
    *                           the above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(Point3DReadOnly, Point3DReadOnly, Vector3DReadOnly, Vector3DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(Point3DReadOnly point,
                                                      Point3DReadOnly pointOnPlane,
                                                      Vector3DReadOnly planeFirstTangent,
                                                      Vector3DReadOnly planeSecondTangent,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;

   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    */
   public static boolean isPoint3DAbovePlane3D(double pointX,
                                               double pointY,
                                               double pointZ,
                                               Point3DReadOnly pointOnPlane,
                                               Vector3DReadOnly planeFirstTangent,
                                               Vector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    */
   public static boolean isPoint3DAbovePlane3D(Point3DReadOnly point,
                                               Point3DReadOnly pointOnPlane,
                                               Vector3DReadOnly planeFirstTangent,
                                               Vector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    */
   public static boolean isPoint3DBelowPlane3D(double pointX,
                                               double pointY,
                                               double pointZ,
                                               Point3DReadOnly pointOnPlane,
                                               Vector3DReadOnly planeFirstTangent,
                                               Vector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    * @see #isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double,
    *      double, double, boolean)
    */
   public static boolean isPoint3DBelowPlane3D(Point3DReadOnly point,
                                               Point3DReadOnly pointOnPlane,
                                               Vector3DReadOnly planeFirstTangent,
                                               Vector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.BELOW;
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Returns a {@code null} if the three points are on a line.
    * <li>Returns {@code null} if two or three points are equal.
    * </ul>
    * </p>
    * <p>
    * The normal is computed such that the points' winding around it is counter-clockwise.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @return the plane normal or {@code null} when the normal could not be determined.
    */
   public static Vector3D normal3DFromThreePoint3Ds(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      Vector3D normal = new Vector3D();
      boolean success = normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normal);
      if (!success)
         return null;
      else
         return normal;
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Fails and returns {@code false} if the three points are on a line.
    * <li>Fails and returns {@code false} if two or three points are equal.
    * </ul>
    * <p>
    * The normal is computed such that the points' winding around it is counter-clockwise.
    * </p>
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @param normalToPack       the vector in which the result is stored. Modified.
    * @return whether the plane normal is properly determined.
    */
   public static boolean normal3DFromThreePoint3Ds(Point3DReadOnly firstPointOnPlane,
                                                   Point3DReadOnly secondPointOnPlane,
                                                   Point3DReadOnly thirdPointOnPlane,
                                                   Vector3DBasics normalToPack)
   {
      double v1_x = secondPointOnPlane.getX() - firstPointOnPlane.getX();
      double v1_y = secondPointOnPlane.getY() - firstPointOnPlane.getY();
      double v1_z = secondPointOnPlane.getZ() - firstPointOnPlane.getZ();

      double v2_x = thirdPointOnPlane.getX() - firstPointOnPlane.getX();
      double v2_y = thirdPointOnPlane.getY() - firstPointOnPlane.getY();
      double v2_z = thirdPointOnPlane.getZ() - firstPointOnPlane.getZ();

      normalToPack.set(v1_y * v2_z - v1_z * v2_y, v2_x * v1_z - v2_z * v1_x, v1_x * v2_y - v1_y * v2_x);

      double normalLength = normalToPack.norm();
      if (normalLength < ONE_TRILLIONTH)
         return false;

      normalToPack.scale(1.0 / normalLength);
      return true;
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLineX     x-coordinate of a point located on the line.
    * @param pointOnLineY     y-coordinate of a point located on the line.
    * @param lineDirectionX   x-component of the direction of the line.
    * @param lineDirectionY   y-component of the direction of the line.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLine2D(Point2DReadOnly pointToProject,
                                                      double pointOnLineX,
                                                      double pointOnLineY,
                                                      double lineDirectionX,
                                                      double lineDirectionY,
                                                      Point2DBasics projectionToPack)
   {
      double directionLengthSquared = normSquared(lineDirectionX, lineDirectionY);

      if (directionLengthSquared < ONE_TRILLIONTH)
         return false;

      double dx = pointToProject.getX() - pointOnLineX;
      double dy = pointToProject.getY() - pointOnLineY;

      double dot = dx * lineDirectionX + dy * lineDirectionY;

      double alpha = dot / directionLengthSquared;

      projectionToPack.set(pointOnLineX + alpha * lineDirectionX, pointOnLineY + alpha * lineDirectionY);

      return true;
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two given points on the line are too close, i.e.
    * {@code firstPointOnLine.distanceSquared(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    */
   public static Point2D orthogonalProjectionOnLine2D(Point2DReadOnly pointToProject, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      Point2D projection = new Point2D();
      boolean success = orthogonalProjectionOnLine2D(pointToProject, firstPointOnLine, secondPointOnLine, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two given points on the line are too close, i.e.
    * {@code firstPointOnLine.distanceSquared(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param projectionToPack  point in which the projection of the point onto the line is stored.
    *                          Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLine2D(Point2DReadOnly pointToProject,
                                                      Point2DReadOnly firstPointOnLine,
                                                      Point2DReadOnly secondPointOnLine,
                                                      Point2DBasics projectionToPack)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      return orthogonalProjectionOnLine2D(pointToProject, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnLine    a point located on the line. Not modified.
    * @param lineDirection  the direction of the line. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    */
   public static Point2D orthogonalProjectionOnLine2D(Point2DReadOnly pointToProject, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      Point2D projection = new Point2D();
      boolean success = orthogonalProjectionOnLine2D(pointToProject, pointOnLine, lineDirection, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the direction of the line. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLine2D(Point2DReadOnly pointToProject,
                                                      Point2DReadOnly pointOnLine,
                                                      Vector2DReadOnly lineDirection,
                                                      Point2DBasics projectionToPack)
   {
      return orthogonalProjectionOnLine2D(pointToProject, pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(), projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on an infinitely long 3D line defined by a 3D
    * point and a 3D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLineX     x-coordinate of a point located on the line.
    * @param pointOnLineY     y-coordinate of a point located on the line.
    * @param pointOnLineZ     z-coordinate of a point located on the line.
    * @param lineDirectionX   x-component of the direction of the line.
    * @param lineDirectionY   y-component of the direction of the line.
    * @param lineDirectionZ   z-component of the direction of the line.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLine3D(Point3DReadOnly pointToProject,
                                                      double pointOnLineX,
                                                      double pointOnLineY,
                                                      double pointOnLineZ,
                                                      double lineDirectionX,
                                                      double lineDirectionY,
                                                      double lineDirectionZ,
                                                      Point3DBasics projectionToPack)
   {
      double directionLengthSquared = normSquared(lineDirectionX, lineDirectionY, lineDirectionZ);

      if (directionLengthSquared < ONE_TRILLIONTH)
         return false;

      double dx = pointToProject.getX() - pointOnLineX;
      double dy = pointToProject.getY() - pointOnLineY;
      double dz = pointToProject.getZ() - pointOnLineZ;

      double dot = dx * lineDirectionX + dy * lineDirectionY + dz * lineDirectionZ;

      double alpha = dot / directionLengthSquared;

      projectionToPack.set(pointOnLineX + alpha * lineDirectionX, pointOnLineY + alpha * lineDirectionY, pointOnLineZ + alpha * lineDirectionZ);
      return true;
   }

   /**
    * Computes the orthogonal projection of a 3D point on an infinitely long 3D line defined by a 3D
    * point and a 3D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnLine    point located on the line. Not modified.
    * @param lineDirection  direction of the line. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    */
   public static Point3D orthogonalProjectionOnLine3D(Point3DReadOnly pointToProject, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      Point3D projection = new Point3D();
      boolean success = orthogonalProjectionOnLine3D(pointToProject, pointOnLine, lineDirection, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 3D point on an infinitely long 3D line defined by a 3D
    * point and a 3D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLine      point located on the line. Not modified.
    * @param lineDirection    direction of the line. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLine3D(Point3DReadOnly pointToProject,
                                                      Point3DReadOnly pointOnLine,
                                                      Vector3DReadOnly lineDirection,
                                                      Point3DBasics projectionToPack)
   {
      return orthogonalProjectionOnLine3D(pointToProject,
                                          pointOnLine.getX(),
                                          pointOnLine.getY(),
                                          pointOnLine.getZ(),
                                          lineDirection.getX(),
                                          lineDirection.getY(),
                                          lineDirection.getZ(),
                                          projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProjectX   the x-coordinate of the point to compute the projection of.
    * @param pointToProjectY   the y-coordinate of the point to compute the projection of.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @param projectionToPack  point in which the projection of the point onto the line segment is
    *                          stored. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(double pointToProjectX,
                                                             double pointToProjectY,
                                                             double lineSegmentStartX,
                                                             double lineSegmentStartY,
                                                             double lineSegmentEndX,
                                                             double lineSegmentEndY,
                                                             Point2DBasics projectionToPack)
   {
      double percentage = percentageAlongLineSegment2D(pointToProjectX,
                                                       pointToProjectY,
                                                       lineSegmentStartX,
                                                       lineSegmentStartY,
                                                       lineSegmentEndX,
                                                       lineSegmentEndY);
      if (percentage > 1.0)
         percentage = 1.0;
      else if (percentage < 0.0)
         percentage = 0.0;

      projectionToPack.set((1.0 - percentage) * lineSegmentStartX + percentage * lineSegmentEndX,
                           (1.0 - percentage) * lineSegmentStartY + percentage * lineSegmentEndY);

      /*
       * This method never fails with the current implementation but the method still returns a boolean in
       * case a failure case is implemented.
       */
      return true;
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @param projectionToPack  point in which the projection of the point onto the line segment is
    *                          stored. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(Point2DReadOnly pointToProject,
                                                             double lineSegmentStartX,
                                                             double lineSegmentStartY,
                                                             double lineSegmentEndX,
                                                             double lineSegmentEndY,
                                                             Point2DBasics projectionToPack)
   {
      return orthogonalProjectionOnLineSegment2D(pointToProject.getX(),
                                                 pointToProject.getY(),
                                                 lineSegmentStartX,
                                                 lineSegmentStartY,
                                                 lineSegmentEndX,
                                                 lineSegmentEndY,
                                                 projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    */
   public static Point2D orthogonalProjectionOnLineSegment2D(Point2DReadOnly pointToProject, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      Point2D projection = new Point2D();
      boolean success = orthogonalProjectionOnLineSegment2D(pointToProject,
                                                            lineSegmentStart.getX(),
                                                            lineSegmentStart.getY(),
                                                            lineSegmentEnd.getX(),
                                                            lineSegmentEnd.getY(),
                                                            projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProjectX  the x-coordinate of the point to compute the projection of.
    * @param pointToProjectY  the y-coordinate of the point to compute the projection of.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(double pointToProjectX,
                                                             double pointToProjectY,
                                                             Point2DReadOnly lineSegmentStart,
                                                             Point2DReadOnly lineSegmentEnd,
                                                             Point2DBasics projectionToPack)
   {
      return orthogonalProjectionOnLineSegment2D(pointToProjectX,
                                                 pointToProjectY,
                                                 lineSegmentStart.getX(),
                                                 lineSegmentStart.getY(),
                                                 lineSegmentEnd.getX(),
                                                 lineSegmentEnd.getY(),
                                                 projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(Point2DReadOnly pointToProject,
                                                             Point2DReadOnly lineSegmentStart,
                                                             Point2DReadOnly lineSegmentEnd,
                                                             Point2DBasics projectionToPack)
   {
      return orthogonalProjectionOnLineSegment2D(pointToProject,
                                                 lineSegmentStart.getX(),
                                                 lineSegmentStart.getY(),
                                                 lineSegmentEnd.getX(),
                                                 lineSegmentEnd.getY(),
                                                 projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D line segment defined by its two 3D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentStartZ the z-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @param lineSegmentEndZ   the z-coordinate of the line segment second endpoint.
    * @param projectionToPack  point in which the projection of the point onto the line segment is
    *                          stored. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLineSegment3D(Point3DReadOnly pointToProject,
                                                             double lineSegmentStartX,
                                                             double lineSegmentStartY,
                                                             double lineSegmentStartZ,
                                                             double lineSegmentEndX,
                                                             double lineSegmentEndY,
                                                             double lineSegmentEndZ,
                                                             Point3DBasics projectionToPack)
   {
      double percentage = percentageAlongLineSegment3D(pointToProject.getX(),
                                                       pointToProject.getY(),
                                                       pointToProject.getZ(),
                                                       lineSegmentStartX,
                                                       lineSegmentStartY,
                                                       lineSegmentStartZ,
                                                       lineSegmentEndX,
                                                       lineSegmentEndY,
                                                       lineSegmentEndZ);
      if (percentage > 1.0)
         percentage = 1.0;
      else if (percentage < 0.0)
         percentage = 0.0;

      projectionToPack.set((1.0 - percentage) * lineSegmentStartX + percentage * lineSegmentEndX,
                           (1.0 - percentage) * lineSegmentStartY + percentage * lineSegmentEndY,
                           (1.0 - percentage) * lineSegmentStartZ + percentage * lineSegmentEndZ);

      /*
       * This method never fails with the current implementation but the method still returns a boolean in
       * case a failure case is implemented.
       */
      return true;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D line segment defined by its two 3D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    */
   public static Point3D orthogonalProjectionOnLineSegment3D(Point3DReadOnly pointToProject, Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      Point3D projection = new Point3D();
      boolean success = orthogonalProjectionOnLineSegment3D(pointToProject,
                                                            lineSegmentStart.getX(),
                                                            lineSegmentStart.getY(),
                                                            lineSegmentStart.getZ(),
                                                            lineSegmentEnd.getX(),
                                                            lineSegmentEnd.getY(),
                                                            lineSegmentEnd.getZ(),
                                                            projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D line segment defined by its two 3D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnLineSegment3D(Point3DReadOnly pointToProject,
                                                             Point3DReadOnly lineSegmentStart,
                                                             Point3DReadOnly lineSegmentEnd,
                                                             Point3DBasics projectionToPack)
   {
      return orthogonalProjectionOnLineSegment3D(pointToProject,
                                                 lineSegmentStart.getX(),
                                                 lineSegmentStart.getY(),
                                                 lineSegmentStart.getZ(),
                                                 lineSegmentEnd.getX(),
                                                 lineSegmentEnd.getY(),
                                                 lineSegmentEnd.getZ(),
                                                 projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than {@link #ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnPlane   a point on the plane. Not modified.
    * @param planeNormal    the normal of the plane. Not modified.
    * @return the projection of the point onto the plane, or {@code null} if the method failed.
    */
   public static Point3D orthogonalProjectionOnPlane3D(Point3DReadOnly pointToProject, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      Point3D projection = new Point3D();
      boolean success = orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than {@link #ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnPlane3D(Point3DReadOnly pointToProject,
                                                       Point3DReadOnly pointOnPlane,
                                                       Vector3DReadOnly planeNormal,
                                                       Point3DBasics projectionToPack)
   {
      return orthogonalProjectionOnPlane3D(pointToProject.getX(), pointToProject.getY(), pointToProject.getZ(), pointOnPlane, planeNormal, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than {@link #ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param x                the x-coordinate of the point to compute the projection of. Not modified.
    * @param y                the y-coordinate of the point to compute the projection of. Not modified.
    * @param z                the z-coordinate of the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean orthogonalProjectionOnPlane3D(double x,
                                                       double y,
                                                       double z,
                                                       Point3DReadOnly pointOnPlane,
                                                       Vector3DReadOnly planeNormal,
                                                       Point3DBasics projectionToPack)
   {
      double normalMagnitude = planeNormal.norm();
      if (normalMagnitude < ONE_TRILLIONTH)
         return false;

      projectionToPack.set(x, y, z);
      projectionToPack.sub(pointOnPlane);
      double signedDistance = projectionToPack.getX() * planeNormal.getX() + projectionToPack.getY() * planeNormal.getY()
                              + projectionToPack.getZ() * planeNormal.getZ();
      signedDistance /= normalMagnitude * normalMagnitude;

      projectionToPack.set(x - signedDistance * planeNormal.getX(), y - signedDistance * planeNormal.getY(), z - signedDistance * planeNormal.getZ());
      return true;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction and returns a percentage {@code alpha} along the first line such that the
    * intersection coordinates can be computed as follows: <br>
    * {@code intersection = pointOnLine1 + alpha * lineDirection1}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and the
    * returned value is {@link Double#NaN}.
    * <li>if the two lines are collinear, this method returns {@link Double#POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1x   x-coordinate of a point located on the first line.
    * @param pointOnLine1y   y-coordinate of a point located on the first line.
    * @param lineDirection1x x-component of the first line direction.
    * @param lineDirection1y y-component of the first line direction.
    * @param pointOnLine2x   x-coordinate of a point located on the second line.
    * @param pointOnLine2y   y-coordinate of a point located on the second line.
    * @param lineDirection2x x-component of the second line direction.
    * @param lineDirection2y y-component of the second line direction.
    * @return {@code alpha} the percentage along the first line of the intersection location. This
    *         method returns {@link Double#NaN} if the lines do not intersect.
    */
   public static double percentageOfIntersectionBetweenTwoLine2Ds(double pointOnLine1x,
                                                                  double pointOnLine1y,
                                                                  double lineDirection1x,
                                                                  double lineDirection1y,
                                                                  double pointOnLine2x,
                                                                  double pointOnLine2y,
                                                                  double lineDirection2x,
                                                                  double lineDirection2y)
   {
      //      We solve for x the problem of the form: A * x = b
      //            A      *     x     =      b
      //      / lineDirection1x -lineDirection2x \   / alpha \   / pointOnLine2x - pointOnLine1x \
      //      |                                  | * |       | = |                               |
      //      \ lineDirection1y -lineDirection2y /   \ beta  /   \ pointOnLine2y - pointOnLine1y /
      // Here, only alpha or beta is needed.

      double determinant = -lineDirection1x * lineDirection2y + lineDirection1y * lineDirection2x;

      double dx = pointOnLine2x - pointOnLine1x;
      double dy = pointOnLine2y - pointOnLine1y;

      if (Math.abs(determinant) < ONE_TRILLIONTH)
      { // The lines are parallel
        // Check if they are collinear
         double cross = dx * lineDirection1y - dy * lineDirection1x;
         if (Math.abs(cross) < ONE_TRILLIONTH)
         {
            /*
             * The two lines are collinear. There's an infinite number of intersection. Let's indicate the
             * result by returning infinity.
             */
            return Double.POSITIVE_INFINITY;
         }
         else
         {
            return Double.NaN;
         }
      }
      else
      {
         double oneOverDeterminant = 1.0 / determinant;
         double AInverse00 = oneOverDeterminant * -lineDirection2y;
         double AInverse01 = oneOverDeterminant * lineDirection2x;

         double alpha = AInverse00 * dx + AInverse01 * dy;

         return alpha;
      }
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction and returns a percentage {@code alpha} along the first line such that the
    * intersection coordinates can be computed as follows: <br>
    * {@code intersection = pointOnLine1 + alpha * lineDirection1}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and the
    * returned value is {@link Double#NaN}.
    * <li>if the two lines are collinear, this method returns {@link Double#POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1   a point located on the first line. Not modified.
    * @param lineDirection1 the first line direction. Not modified.
    * @param pointOnLine2   a point located on the second line. Not modified.
    * @param lineDirection2 the second line direction. Not modified.
    * @return {@code alpha} the percentage along the first line of the intersection location. This
    *         method returns {@link Double#NaN} if the lines do not intersect.
    */
   public static double percentageOfIntersectionBetweenTwoLine2Ds(Point2DReadOnly pointOnLine1,
                                                                  Vector2DReadOnly lineDirection1,
                                                                  Point2DReadOnly pointOnLine2,
                                                                  Vector2DReadOnly lineDirection2)
   {
      return percentageOfIntersectionBetweenTwoLine2Ds(pointOnLine1.getX(),
                                                       pointOnLine1.getY(),
                                                       lineDirection1.getX(),
                                                       lineDirection1.getY(),
                                                       pointOnLine2.getX(),
                                                       pointOnLine2.getY(),
                                                       lineDirection2.getX(),
                                                       lineDirection2.getY());
   }

   /**
    * Computes the intersection between a 2D line segment and an infinitely long 2D line and returns a
    * percentage {@code alpha} along the line segment such that the intersection coordinates can be
    * computed as follows: <br>
    * {@code intersection = (1.0 - alpha) * lineSegmentStart + alpha * lineSegmentEnd}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line segment and the line do not intersect, the method returns {@link Double#NaN}.
    * <li>if the intersection is outside the line segment's endpoints, the line segment and the line do
    * not intersect.
    * <li>if the line segment and the line are parallel but not collinear, they do not intersect and
    * the returned value is {@link Double#NaN}.
    * <li>if the line segment and the line are collinear, this method returns
    * {@link Double#POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the line direction. Not modified.
    * @return {@code alpha} the percentage along the line segment of the intersection location. This
    *         method returns {@link Double#NaN} if the line segment and the line do not intersect.
    */
   public static double percentageOfIntersectionBetweenLineSegment2DAndLine2D(Point2DReadOnly lineSegmentStart,
                                                                              Point2DReadOnly lineSegmentEnd,
                                                                              Point2DReadOnly pointOnLine,
                                                                              Vector2DReadOnly lineDirection)
   {
      double lineSegmentStartX = lineSegmentStart.getX();
      double lineSegmentStartY = lineSegmentStart.getY();
      double lineSegmentDirectionX = lineSegmentEnd.getX() - lineSegmentStart.getX();
      double lineSegmentDirectionY = lineSegmentEnd.getY() - lineSegmentStart.getY();
      double alpha = percentageOfIntersectionBetweenTwoLine2Ds(lineSegmentStartX,
                                                               lineSegmentStartY,
                                                               lineSegmentDirectionX,
                                                               lineSegmentDirectionY,
                                                               pointOnLine.getX(),
                                                               pointOnLine.getY(),
                                                               lineDirection.getX(),
                                                               lineDirection.getY());
      if (Double.isInfinite(alpha))
         return Double.POSITIVE_INFINITY;
      if (Double.isNaN(alpha) || alpha < 0.0 - ONE_TEN_MILLIONTH || alpha > 1.0 + ONE_TEN_MILLIONTH)
         return Double.NaN;
      else if (alpha < 0.0)
         return 0.0;
      else if (alpha > 1.0)
         return 1.0;
      else
         return alpha;
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point         the coordinates of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    */
   public static double percentageAlongLine2D(Point2DReadOnly point, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return percentageAlongLine2D(point.getX(), point.getY(), pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY());
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    */
   public static double percentageAlongLine2D(double pointX, double pointY, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return percentageAlongLine2D(pointX, pointY, pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY());
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX         the x-coordinate of the query point.
    * @param pointY         the y-coordinate of the query point.
    * @param pointOnLineX   x-coordinate of a point located on the line.
    * @param pointOnLineY   y-coordinate of a point located on the line.
    * @param lineDirectionX x-component of the direction of the line.
    * @param lineDirectionY y-component of the direction of the line.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    */
   public static double percentageAlongLine2D(double pointX,
                                              double pointY,
                                              double pointOnLineX,
                                              double pointOnLineY,
                                              double lineDirectionX,
                                              double lineDirectionY)
   {
      double lengthSquared = normSquared(lineDirectionX, lineDirectionY);

      if (lengthSquared < ONE_TRILLIONTH)
         return 0.0;

      double dx = pointX - pointOnLineX;
      double dy = pointY - pointOnLineY;

      double dot = dx * lineDirectionX + dy * lineDirectionY;

      return dot / lengthSquared;
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto the line segment. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX            the x-coordinate of the query point.
    * @param pointY            the y-coordinate of the query point.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   public static double percentageAlongLineSegment2D(double pointX,
                                                     double pointY,
                                                     double lineSegmentStartX,
                                                     double lineSegmentStartY,
                                                     double lineSegmentEndX,
                                                     double lineSegmentEndY)
   {
      double lineSegmentDx = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDy = lineSegmentEndY - lineSegmentStartY;
      double lengthSquared = normSquared(lineSegmentDx, lineSegmentDy);

      if (lengthSquared < ONE_TRILLIONTH)
         return 0.0;

      double dx = pointX - lineSegmentStartX;
      double dy = pointY - lineSegmentStartY;

      double dot = dx * lineSegmentDx + dy * lineSegmentDy;

      return dot / lengthSquared;
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX           the x-coordinate of the query point.
    * @param pointY           the y-coordinate of the query point.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   public static double percentageAlongLineSegment2D(double pointX, double pointY, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return percentageAlongLineSegment2D(pointX, pointY, lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point            the query. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   public static double percentageAlongLineSegment2D(Point2DReadOnly point, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return percentageAlongLineSegment2D(point.getX(), point.getY(), lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point         the coordinates of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    */
   public static double percentageAlongLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return percentageAlongLine3D(point.getX(),
                                   point.getY(),
                                   point.getZ(),
                                   pointOnLine.getX(),
                                   pointOnLine.getY(),
                                   pointOnLine.getZ(),
                                   lineDirection.getX(),
                                   lineDirection.getY(),
                                   lineDirection.getZ());
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointZ        the z-coordinate of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    */
   public static double percentageAlongLine3D(double pointX, double pointY, double pointZ, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return percentageAlongLine3D(pointX,
                                   pointY,
                                   pointZ,
                                   pointOnLine.getX(),
                                   pointOnLine.getY(),
                                   pointOnLine.getZ(),
                                   lineDirection.getX(),
                                   lineDirection.getY(),
                                   lineDirection.getZ());
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX         the x-coordinate of the query point.
    * @param pointY         the y-coordinate of the query point.
    * @param pointZ         the z-coordinate of the query point.
    * @param pointOnLineX   x-coordinate of a point located on the line.
    * @param pointOnLineY   y-coordinate of a point located on the line.
    * @param pointOnLineZ   z-coordinate of a point located on the line.
    * @param lineDirectionX x-component of the direction of the line.
    * @param lineDirectionY y-component of the direction of the line.
    * @param lineDirectionZ z-component of the direction of the line.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    */
   public static double percentageAlongLine3D(double pointX,
                                              double pointY,
                                              double pointZ,
                                              double pointOnLineX,
                                              double pointOnLineY,
                                              double pointOnLineZ,
                                              double lineDirectionX,
                                              double lineDirectionY,
                                              double lineDirectionZ)
   {
      double lengthSquared = normSquared(lineDirectionX, lineDirectionY, lineDirectionZ);

      if (lengthSquared < ONE_TRILLIONTH)
         return 0.0;

      double dx = pointX - pointOnLineX;
      double dy = pointY - pointOnLineY;
      double dz = pointZ - pointOnLineZ;

      double dot = dx * lineDirectionX + dy * lineDirectionY + dz * lineDirectionZ;

      return dot / lengthSquared;
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto the line segment. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point3DReadOnly projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX            the x-coordinate of the query point.
    * @param pointY            the y-coordinate of the query point.
    * @param pointZ            the z-coordinate of the query point.
    * @param lineSegmentStartX the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY the y-coordinate of the line segment first endpoint.
    * @param lineSegmentStartZ the z-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX   the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY   the y-coordinate of the line segment second endpoint.
    * @param lineSegmentEndZ   the z-coordinate of the line segment second endpoint.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   public static double percentageAlongLineSegment3D(double pointX,
                                                     double pointY,
                                                     double pointZ,
                                                     double lineSegmentStartX,
                                                     double lineSegmentStartY,
                                                     double lineSegmentStartZ,
                                                     double lineSegmentEndX,
                                                     double lineSegmentEndY,
                                                     double lineSegmentEndZ)
   {
      double lineSegmentDx = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDy = lineSegmentEndY - lineSegmentStartY;
      double lineSegmentDz = lineSegmentEndZ - lineSegmentStartZ;
      double lengthSquared = normSquared(lineSegmentDx, lineSegmentDy, lineSegmentDz);

      if (lengthSquared < ONE_TRILLIONTH)
         return 0.0;

      double dx = pointX - lineSegmentStartX;
      double dy = pointY - lineSegmentStartY;
      double dz = pointZ - lineSegmentStartZ;

      double dot = dx * lineSegmentDx + dy * lineSegmentDy + dz * lineSegmentDz;

      return dot / lengthSquared;
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point3DReadOnly projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX           the x-coordinate of the query point.
    * @param pointY           the y-coordinate of the query point.
    * @param pointZ           the z-coordinate of the query point.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   public static double percentageAlongLineSegment3D(double pointX,
                                                     double pointY,
                                                     double pointZ,
                                                     Point3DReadOnly lineSegmentStart,
                                                     Point3DReadOnly lineSegmentEnd)
   {
      return percentageAlongLineSegment3D(pointX,
                                          pointY,
                                          pointZ,
                                          lineSegmentStart.getX(),
                                          lineSegmentStart.getY(),
                                          lineSegmentStart.getZ(),
                                          lineSegmentEnd.getX(),
                                          lineSegmentEnd.getY(),
                                          lineSegmentEnd.getZ());
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point3DReadOnly projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point            the query. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   public static double percentageAlongLineSegment3D(Point3DReadOnly point, Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      return percentageAlongLineSegment3D(point.getX(), point.getY(), point.getZ(), lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes the perpendicular bisector of line segment defined by its two endpoints. The bisector
    * starts off the the middle of the line segment and points toward the left side of the line
    * segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, the method fails
    * and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart        the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd          the second endpoint of the line segment. Not modified.
    * @param bisectorStartToPack     a 2D point in which the origin of the bisector is stored.
    *                                Modified.
    * @param bisectorDirectionToPack a 2D vector in which the direction of the bisector is stored.
    *                                Modified.
    * @return whether the perpendicular bisector could be determined or not.
    */
   public static boolean perpendicularBisector2D(Point2DReadOnly lineSegmentStart,
                                                 Point2DReadOnly lineSegmentEnd,
                                                 Point2DBasics bisectorStartToPack,
                                                 Vector2DBasics bisectorDirectionToPack)
   {
      if (lineSegmentStart.distance(lineSegmentEnd) < ONE_TRILLIONTH)
         return false;

      // direction will be on left side of line
      bisectorStartToPack.interpolate(lineSegmentStart, lineSegmentEnd, 0.5);
      bisectorDirectionToPack.sub(lineSegmentEnd, lineSegmentStart);
      perpendicularVector2D(bisectorDirectionToPack, bisectorDirectionToPack);
      bisectorDirectionToPack.normalize();
      return true;
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its
    * endpoints, such that:
    * <ul>
    * <li>each endpoint of the perpendicular bisector is at a distance of
    * {@code bisectorSegmentHalfLength} from the line segment.
    * <li>the first perpendicular bisector endpoint is located on the left side on the line segment.
    * <li>the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, the method fails
    * and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param lineSegmentStart          the first endpoint of the line segment from which the
    *                                  perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd            the second endpoint of the line segment from which the
    *                                  perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength distance from the line segment each endpoint of the
    *                                  perpendicular bisector segment will be positioned.
    * @return a list containing the two endpoints of the perpendicular bisector segment.
    */
   public static List<Point2D> perpendicularBisectorSegment2D(Point2DReadOnly lineSegmentStart,
                                                              Point2DReadOnly lineSegmentEnd,
                                                              double bisectorSegmentHalfLength)
   {
      Point2D bisectorSegmentStart = new Point2D();
      Point2D bisectorSegmentEnd = new Point2D();

      boolean success = perpendicularBisectorSegment2D(lineSegmentStart, lineSegmentEnd, bisectorSegmentHalfLength, bisectorSegmentStart, bisectorSegmentEnd);
      if (!success)
         return null;

      List<Point2D> bisectorEndpoints = new ArrayList<>();
      bisectorEndpoints.add(bisectorSegmentStart);
      bisectorEndpoints.add(bisectorSegmentEnd);
      return bisectorEndpoints;
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its
    * endpoints, such that:
    * <ul>
    * <li>each endpoint of the perpendicular bisector is at a distance of
    * {@code bisectorSegmentHalfLength} from the line segment.
    * <li>the first perpendicular bisector endpoint is located on the left side on the line segment.
    * <li>the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, the method fails
    * and returns false.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart           the first endpoint of the line segment from which the
    *                                   perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd             the second endpoint of the line segment from which the
    *                                   perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength  distance from the line segment each endpoint of the
    *                                   perpendicular bisector segment will be positioned.
    * @param bisectorSegmentStartToPack the first endpoint of the perpendicular bisector segment to be
    *                                   computed. Modified.
    * @param bisectorSegmentEndToPack   the second endpoint of the perpendicular bisector segment to be
    *                                   computed. Modified.
    * @return whether the perpendicular bisector could be determined or not.
    */
   public static boolean perpendicularBisectorSegment2D(Point2DReadOnly lineSegmentStart,
                                                        Point2DReadOnly lineSegmentEnd,
                                                        double bisectorSegmentHalfLength,
                                                        Point2DBasics bisectorSegmentStartToPack,
                                                        Point2DBasics bisectorSegmentEndToPack)
   {
      if (lineSegmentStart.distance(lineSegmentEnd) < ONE_TRILLIONTH)
         return false;

      // direction will be on left side of line
      double bisectorDirectionX = -(lineSegmentEnd.getY() - lineSegmentStart.getY());
      double bisectorDirectionY = lineSegmentEnd.getX() - lineSegmentStart.getX();
      double directionInverseMagnitude = 1.0 / EuclidCoreTools.norm(bisectorDirectionX, bisectorDirectionY);
      bisectorDirectionX *= directionInverseMagnitude;
      bisectorDirectionY *= directionInverseMagnitude;

      double midPointX = 0.5 * (lineSegmentStart.getX() + lineSegmentEnd.getX());
      double midPointY = 0.5 * (lineSegmentStart.getY() + lineSegmentEnd.getY());

      bisectorSegmentStartToPack.set(midPointX + bisectorDirectionX * bisectorSegmentHalfLength, midPointY + bisectorDirectionY * bisectorSegmentHalfLength);
      bisectorSegmentEndToPack.set(midPointX - bisectorDirectionX * bisectorSegmentHalfLength, midPointY - bisectorDirectionY * bisectorSegmentHalfLength);

      return true;
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    * <li>{@code vector.dot(perpendicularVector) == 0.0}.
    * <li>{@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vector the vector to compute the perpendicular of. Not modified.
    * @return the perpendicular vector.
    */
   public static Vector2D perpendicularVector2D(Vector2DReadOnly vector)
   {
      return new Vector2D(-vector.getY(), vector.getX());
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    * <li>{@code vector.dot(perpendicularVector) == 0.0}.
    * <li>{@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    *
    * @param vector                    the vector to compute the perpendicular of. Not modified.
    * @param perpendicularVectorToPack a 2D vector in which the perpendicular vector is stored.
    *                                  Modified.
    */
   public static void perpendicularVector2D(Vector2DReadOnly vector, Vector2DBasics perpendicularVectorToPack)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX());
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below {@value #ONE_TRILLIONTH},
    * the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @return the vector perpendicular to the line and pointing to the {@code point}, or {@code null}
    *         when the method fails.
    */
   public static Vector3D perpendicularVector3DFromLine3DToPoint3D(Point3DReadOnly point,
                                                                   Point3DReadOnly firstPointOnLine,
                                                                   Point3DReadOnly secondPointOnLine,
                                                                   Point3DBasics orthogonalProjectionToPack)
   {
      Vector3D perpendicularVector = new Vector3D();
      boolean success = perpendicularVector3DFromLine3DToPoint3D(point, firstPointOnLine, secondPointOnLine, orthogonalProjectionToPack, perpendicularVector);
      if (!success)
         return null;
      else
         return perpendicularVector;
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below {@value #ONE_TRILLIONTH},
    * the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @param perpendicularVectorToPack  a 3D vector in which the vector perpendicular to the line and
    *                                   pointing to the {@code point} is stored. Modified. Can NOT be
    *                                   {@code null}.
    * @return {@code true} if the method succeeded, {@code false} otherwise.
    */
   public static boolean perpendicularVector3DFromLine3DToPoint3D(Point3DReadOnly point,
                                                                  Point3DReadOnly firstPointOnLine,
                                                                  Point3DReadOnly secondPointOnLine,
                                                                  Point3DBasics orthogonalProjectionToPack,
                                                                  Vector3DBasics perpendicularVectorToPack)
   {
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double lineDirectionZ = secondPointOnLine.getZ() - firstPointOnLine.getZ();
      double lineLength = EuclidCoreTools.fastNorm(lineDirectionX, lineDirectionY, lineDirectionZ);

      if (lineLength < ONE_TRILLIONTH)
         return false;

      lineLength = 1.0 / lineLength;
      lineDirectionX *= lineLength;
      lineDirectionY *= lineLength;
      lineDirectionZ *= lineLength;

      double dx = point.getX() - firstPointOnLine.getX();
      double dy = point.getY() - firstPointOnLine.getY();
      double dz = point.getZ() - firstPointOnLine.getZ();

      double distanceFromFirstPointOnLine = lineDirectionX * dx + lineDirectionY * dy + lineDirectionZ * dz;

      if (orthogonalProjectionToPack != null)
      {
         orthogonalProjectionToPack.set(lineDirectionX, lineDirectionY, lineDirectionZ);
         orthogonalProjectionToPack.scaleAdd(distanceFromFirstPointOnLine, orthogonalProjectionToPack, firstPointOnLine);
         perpendicularVectorToPack.sub(point, orthogonalProjectionToPack);
      }
      else
      {
         perpendicularVectorToPack.set(lineDirectionX, lineDirectionY, lineDirectionZ);
         perpendicularVectorToPack.scale(distanceFromFirstPointOnLine);
         perpendicularVectorToPack.add(firstPointOnLine);
         perpendicularVectorToPack.negate();
         perpendicularVectorToPack.add(point);
      }
      return true;
   }

   /**
    * Get a unknown cathetus (90-deg triangle one of the two shorter triangle sides, neighbouring the
    * 90-degree angle) by Pythagoras law.
    * <p>
    * Given a right triangle with the three sides A, B, and C, where A and B are the catheti and C the
    * hypotenuse, this method calculates the length of the cathetus B given the lengths of A and C:
    * <br>
    * |B|<sup>2</sup> = |C|<sup>2</sup> - |A|<sup>2</sup>. </br>
    * <a href="https://en.wikipedia.org/wiki/Cathetus"> Useful link</a>.
    * </p>
    *
    * @param hypotenuseC the length of the hypotenuse C.
    * @param cathetusA   the length of the cathetus A.
    * @return the length of the cathetus B.
    * @throws IllegalArgumentException if the length of the cathetus A is negative or greater than the
    *                                  hypotenuse C.
    */
   public static double pythagorasGetCathetus(double hypotenuseC, double cathetusA)
   {
      if (hypotenuseC < 0.0)
         throw new IllegalArgumentException("The hypotenuse cannot have a negative length, hypotenuseC = " + hypotenuseC);
      if (cathetusA < 0.0)
         throw new IllegalArgumentException("The cathetus cannot have a negative length, cathetusA = " + cathetusA);
      if (cathetusA > hypotenuseC)
         throw new IllegalArgumentException("The cathetus cannot be longer than the hypotenuse, cathetusA = " + cathetusA + ", hypotenuseC = " + hypotenuseC);

      return EuclidCoreTools.squareRoot(hypotenuseC * hypotenuseC - cathetusA * cathetusA);
   }

   /**
    * Get the hypotenuse c (90-degree triangle longest triangle length, opposite to the 90-degree
    * angle) by Pythagoras law, a^2+b^2=c^2
    * <p>
    * Given a right triangle with the three sides A, B, and C, where A and B are the catheti and C the
    * hypotenuse, this method calculates the length of the hypotenuse C given the lengths of A and B:
    * <br>
    * |C|<sup>2</sup> = |A|<sup>2</sup> + |B|<sup>2</sup>. </br>
    * <a href="https://en.wikipedia.org/wiki/Cathetus"> Useful link</a>.
    * </p>
    *
    * @param cathetusA the length of the cathetus A.
    * @param cathetusB the length of the cathetus B.
    * @return the length of the hypotenuse C.
    * @throws IllegalArgumentException if any of the two lengths is negative.
    */
   public static double pythagorasGetHypotenuse(double cathetusA, double cathetusB)
   {
      if (cathetusA < 0.0)
         throw new IllegalArgumentException("The cathetus A cannot have a negative length, cathetusA = " + cathetusA);
      if (cathetusB < 0.0)
         throw new IllegalArgumentException("The cathetus B cannot have a negative length, cathetusB = " + cathetusB);
      return Math.hypot(cathetusA, cathetusB);
   }

   /**
    * Returns the radius of an arc with the specified chord length and angle.
    * <a href="http://planetcalc.com/1421/"> Useful link</a>.
    *
    * @param chordLength the length of the chord.
    * @param chordAngle  angle covered by the chord.
    * @return the radius of the arc, or {@code Double.NaN} if {@code chordAngle % Math.PI == 0.0}.
    */
   public static double radiusOfArc(double chordLength, double chordAngle)
   {
      if (chordAngle % Math.PI == 0.0)
         return Double.NaN;
      else
         return chordLength / (2.0 * EuclidCoreTools.sin(0.5 * chordAngle));
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code firstPointOnLine.distance(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x-coordinate of the query.
    * @param pointY            y-coordinate of the query.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    */
   public static double signedDistanceFromPoint2DToLine2D(double pointX, double pointY, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      return signedDistanceFromPoint2DToLine2D(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX        x-coordinate of the query.
    * @param pointY        y-coordinate of the query.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    */
   public static double signedDistanceFromPoint2DToLine2D(double pointX, double pointY, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return signedDistanceFromPoint2DToLine2D(pointX,
                                               pointY,
                                               pointOnLine.getX(),
                                               pointOnLine.getY(),
                                               lineDirection.getX(),
                                               lineDirection.getY(),
                                               lineDirection instanceof UnitVector2DReadOnly);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code firstPointOnLine.distance(secondPointOnLine) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point             the coordinates of the query. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    */
   public static double signedDistanceFromPoint2DToLine2D(Point2DReadOnly point, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      return signedDistanceFromPoint2DToLine2D(point.getX(), point.getY(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    */
   public static double signedDistanceFromPoint2DToLine2D(Point2DReadOnly point, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return signedDistanceFromPoint2DToLine2D(point.getX(), point.getY(), pointOnLine, lineDirection);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX         x-coordinate of the query.
    * @param pointY         y-coordinate of the query.
    * @param pointOnLineX   x-coordinate of a point located on the line.
    * @param pointOnLineY   y-coordinate of a point located on the line.
    * @param lineDirectionX x-component of the line direction.
    * @param lineDirectionY y-component of the line direction.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    */
   public static double signedDistanceFromPoint2DToLine2D(double pointX,
                                                          double pointY,
                                                          double pointOnLineX,
                                                          double pointOnLineY,
                                                          double lineDirectionX,
                                                          double lineDirectionY)
   {
      return signedDistanceFromPoint2DToLine2D(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, false);
   }

   private static double signedDistanceFromPoint2DToLine2D(double pointX,
                                                           double pointY,
                                                           double pointOnLineX,
                                                           double pointOnLineY,
                                                           double lineDirectionX,
                                                           double lineDirectionY,
                                                           boolean isDirectionUnitary)
   {
      double dx = pointX - pointOnLineX;
      double dy = pointY - pointOnLineY;
      double directionMagnitude = isDirectionUnitary ? 1.0 : EuclidCoreTools.fastNorm(lineDirectionX, lineDirectionY);

      if (directionMagnitude < ONE_TRILLIONTH)
      {
         return EuclidCoreTools.norm(dx, dy);
      }
      else
      {
         return (lineDirectionX * dy - dx * lineDirectionY) / directionMagnitude;
      }
   }

   /**
    * Computes the position of the sphere given its radius and three points that lie on its surface.
    * <p>
    * There two possible solutions to this problem. The solution returned is located "above" the
    * triangle's plane. "Above" is defined by the direction given by the normal of the three points
    * such as their winding is counter-clockwise.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>In the case the radius of the sphere is too small to reach all three points, i.e. the radius
    * is smaller than the circumradius of the triangle, this method fails and returns {@code false}.
    * <li>If the problem is degenerate, i.e. any of the three lengths of the triangle is zero, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param p1                     the first point that belongs to the sphere. Not modified.
    * @param p2                     the second point that belongs to the sphere. Not modified.
    * @param p3                     the third point that belongs to the sphere. Not modified.
    * @param sphere3DRadius         the radius of the sphere.
    * @param sphere3DPositionToPack the point used to store the result. Modified.
    * @return whether the sphere position was successfully or not.
    */
   public static boolean sphere3DPositionFromThreePoints(Point3DReadOnly p1,
                                                         Point3DReadOnly p2,
                                                         Point3DReadOnly p3,
                                                         double sphere3DRadius,
                                                         Point3DBasics sphere3DPositionToPack)
   {
      double ux = p2.getX() - p1.getX();
      double uy = p2.getY() - p1.getY();
      double uz = p2.getZ() - p1.getZ();

      double vx = p3.getX() - p1.getX();
      double vy = p3.getY() - p1.getY();
      double vz = p3.getZ() - p1.getZ();

      double wx = uy * vz - uz * vy;
      double wy = uz * vx - ux * vz;
      double wz = ux * vy - uy * vx;

      double uSquared = EuclidCoreTools.normSquared(ux, uy, uz);
      double vSquared = EuclidCoreTools.normSquared(vx, vy, vz);
      double wSquaredInverse = 1.0 / EuclidCoreTools.normSquared(wx, wy, wz);
      double uv = ux * vx + uy * vy + uz * vz;

      double alpha = 0.5 * (uSquared - uv) * vSquared * wSquaredInverse;
      double beta = 0.5 * (vSquared - uv) * uSquared * wSquaredInverse;

      double radiusSquared = sphere3DRadius * sphere3DRadius;
      double alphaSquared = alpha * alpha;
      double betaSquared = beta * beta;

      double gammaSquared = (radiusSquared - alphaSquared * uSquared - betaSquared * vSquared - 2.0 * alpha * beta * uv) * wSquaredInverse;

      if (gammaSquared < 0.0 || !Double.isFinite(gammaSquared))
         return false; // The radius of the sphere is too small, or the problem is degenerate.

      double gamma = EuclidCoreTools.squareRoot(gammaSquared);

      double px = p1.getX() + alpha * ux + beta * vx + gamma * wx;
      double py = p1.getY() + alpha * uy + beta * vy + gamma * wy;
      double pz = p1.getZ() + alpha * uz + beta * vz + gamma * wz;
      sphere3DPositionToPack.set(px, py, pz);
      return true;
   }

   /**
    * Assuming an isosceles triangle defined by three vertices A, B, and C, with |AB| == |BC|, this
    * methods computes the missing vertex B given the vertices A and C, the normal of the triangle, the
    * angle ABC that is equal to the angle at B from the the leg BA to the leg BC.
    * <a href="https://en.wikipedia.org/wiki/Isosceles_triangle"> Useful link</a>.
    *
    * @param baseVertexA                    the first base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param baseVertexC                    the second base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param trianglePlaneNormal            the normal of the plane on which is lying. Not modified.
    * @param ccwAngleAboutNormalAtTopVertex the angle at B from the the leg BA to the leg BC.
    * @param topVertexBToPack               the missing vertex B. Modified.
    */
   public static void topVertex3DOfIsoscelesTriangle3D(Point3DReadOnly baseVertexA,
                                                       Point3DReadOnly baseVertexC,
                                                       Vector3DReadOnly trianglePlaneNormal,
                                                       double ccwAngleAboutNormalAtTopVertex,
                                                       Point3DBasics topVertexBToPack)
   {
      double baseEdgeACx = baseVertexC.getX() - baseVertexA.getX();
      double baseEdgeACy = baseVertexC.getY() - baseVertexA.getY();
      double baseEdgeACz = baseVertexC.getZ() - baseVertexA.getZ();
      double baseEdgeACLength = EuclidCoreTools.norm(baseEdgeACx, baseEdgeACy, baseEdgeACz);

      double legLengthABorCB = radiusOfArc(baseEdgeACLength, ccwAngleAboutNormalAtTopVertex);
      double lengthOfBisectorOfBase = pythagorasGetCathetus(legLengthABorCB, 0.5 * baseEdgeACLength);

      double perpendicularBisectorX = trianglePlaneNormal.getY() * baseEdgeACz - trianglePlaneNormal.getZ() * baseEdgeACy;
      double perpendicularBisectorY = trianglePlaneNormal.getZ() * baseEdgeACx - trianglePlaneNormal.getX() * baseEdgeACz;
      double perpendicularBisectorZ = trianglePlaneNormal.getX() * baseEdgeACy - trianglePlaneNormal.getY() * baseEdgeACx;
      double scale = lengthOfBisectorOfBase;
      scale /= EuclidCoreTools.norm(perpendicularBisectorX, perpendicularBisectorY, perpendicularBisectorZ);
      perpendicularBisectorX *= scale;
      perpendicularBisectorY *= scale;
      perpendicularBisectorZ *= scale;

      topVertexBToPack.interpolate(baseVertexA, baseVertexC, 0.5);
      topVertexBToPack.set(topVertexBToPack.getX() + perpendicularBisectorX,
                           topVertexBToPack.getY() + perpendicularBisectorY,
                           topVertexBToPack.getZ() + perpendicularBisectorZ);
   }

   /**
    * Given a triangle defined by three points (A,B,C), this methods the point X &in; AC such that the
    * line (B, X) is the angle bisector of B. As a result, the two angles CBX and XBA are equal.
    * <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if any the triangle's edge is shorter than {@link #ONE_TRILLIONTH}, this method fails and
    * returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param A the first vertex of the triangle. Not modified.
    * @param B the second vertex of the triangle, this is the first endpoint of the bisector. Not
    *          modified.
    * @param C the third vertex of the triangle. Not modified.
    * @return the second endpoint of the bisector, or {@code null} if the method failed.
    */
   public static Point2D triangleBisector2D(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C)
   {
      Point2D X = new Point2D();
      boolean success = triangleBisector2D(A, B, C, X);
      if (success)
         return X;
      else
         return null;
   }

   /**
    * Given a triangle defined by three points (A,B,C), this methods the point X &in; AC such that the
    * line (B, X) is the angle bisector of B. As a result, the two angles CBX and XBA are equal.
    * <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if any the triangle's edge is shorter than {@link #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param A       the first vertex of the triangle. Not modified.
    * @param B       the second vertex of the triangle, this is the first endpoint of the bisector. Not
    *                modified.
    * @param C       the third vertex of the triangle. Not modified.
    * @param XToPack point in which the second endpoint of the bisector is stored. Modified.
    * @return whether the bisector could be calculated or not.
    */
   public static boolean triangleBisector2D(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DBasics XToPack)
   {
      // find all proportional values
      double BA = B.distance(A);
      if (BA < ONE_TRILLIONTH)
         return false;

      double BC = B.distance(C);
      if (BC < ONE_TRILLIONTH)
         return false;

      double AC = A.distance(C);

      if (AC < ONE_TRILLIONTH)
         return false;

      double AX = AC / (BC / BA + 1.0);

      // use AX distance to find X along AC
      double vectorAXx = C.getX() - A.getX();
      double vectorAXy = C.getY() - A.getY();
      double inverseMagnitude = 1.0 / EuclidCoreTools.norm(vectorAXx, vectorAXy);
      vectorAXx *= AX * inverseMagnitude;
      vectorAXy *= AX * inverseMagnitude;

      XToPack.set(vectorAXx, vectorAXy);
      XToPack.add(A);
      return true;
   }

   /**
    * Computes the radius of the circumcircle of the triangle ABC given the length of its edges.
    *
    * @param lengthA the length of the side A of the triangle.
    * @param lengthB the length of the side B of the triangle.
    * @param lengthC the length of the side C of the triangle.
    * @return the circumradius.
    */
   public static double triangleCircumradius(double lengthA, double lengthB, double lengthC)
   {
      // From https://en.wikipedia.org/wiki/Triangle#Circumradius_and_inradius
      double circumradiusSquared = lengthA * lengthA * lengthB * lengthB * lengthC * lengthC;
      circumradiusSquared /= (lengthA + lengthB + lengthC) * (-lengthA + lengthB + lengthC) * (lengthA - lengthB + lengthC) * (lengthA + lengthB - lengthC);
      return EuclidCoreTools.squareRoot(circumradiusSquared);
   }

   /**
    * Computes the length of the height of the triangle from its sides lengths.
    * <p>
    * The height is the line that originates from the triangle's base to end at the triangle's apex
    * (vertex on the opposite side of the base).
    * </p>
    *
    * @param legLength  the length of the sides that are of same length.
    * @param baseLength the length of the side that has different length from the other sides.
    * @return the length of the height.
    */
   public static double triangleIsoscelesHeight(double legLength, double baseLength)
   {
      if (legLength < 0.5 * baseLength)
         throw new IllegalArgumentException("Malformed isosceles triangle, expected legLength > baseLength/2, was legLength: " + legLength + ", baseLength/2: "
                                            + 0.5 * baseLength);
      if (baseLength < 0.0)
         throw new IllegalArgumentException("The base cannot have a negative length, baseLength = " + baseLength);

      return EuclidCoreTools.squareRoot(legLength * legLength - 0.25 * baseLength * baseLength);
   }

   /**
    * Calculate an unknown angle of a fully defined 2D Triangle by the law of Cosine.
    * <p>
    * Given a triangle with the three sides A, B, and C, this methods calculates the angle between A
    * and B given the lengths of three sides.
    *
    * @param lengthNeighbourSideA the length of the side A.
    * @param lengthNeighbourSideB the length of the side B.
    * @param lengthOppositeSideC  the length of the side C.
    * @return the value in radians of the unknown angle.
    * @throws IllegalArgumentException if the lengths do not describe a triangle, see
    *                                  {@link #isFormingTriangle(double, double, double)}.
    */
   public static double unknownTriangleAngleByLawOfCosine(double lengthNeighbourSideA, double lengthNeighbourSideB, double lengthOppositeSideC)
   {
      if (!isFormingTriangle(lengthNeighbourSideA, lengthNeighbourSideB, lengthOppositeSideC))
      {
         throw new IllegalArgumentException("Unable to build a Triangle of the given triangle sides a: " + lengthNeighbourSideA + " b: " + lengthNeighbourSideB
                                            + " c: " + lengthOppositeSideC);
      }

      double numerator = lengthNeighbourSideA * lengthNeighbourSideA + lengthNeighbourSideB * lengthNeighbourSideB - lengthOppositeSideC * lengthOppositeSideC;
      double denominator = 2.0 * lengthNeighbourSideA * lengthNeighbourSideB;
      return EuclidCoreTools.fastAcos(numerator / denominator);
   }

   /**
    * Calculate an unknown side length of a fully defined 2D Triangle by the law of Cosine.
    * <p>
    * Given a triangle with the three sides A, B, and C, this methods calculates the length of the side
    * C, given:
    * <ul>
    * <li>the lengths of A and B.
    * <li>the angle between the sides A and B.
    * </ul>
    * </p>
    *
    * @param lengthSideA       the length of the side A.
    * @param lengthSideB       the length of the side B.
    * @param angleBetweenAAndB the angle between the sides A and B.
    * @return the value of the unknown side length.
    * @throws IllegalArgumentException if {@code lengthSideA} and/or {@code lengthSideB} are negative,
    *                                  if {@code angleBetweenAAndB} is greater than <i>pi</i>.
    */
   public static double unknownTriangleSideLengthByLawOfCosine(double lengthSideA, double lengthSideB, double angleBetweenAAndB)
   {
      if (lengthSideA < 0.0)
         throw new IllegalArgumentException("lengthSideA cannot be negative: " + lengthSideA);
      if (lengthSideB < 0.0)
         throw new IllegalArgumentException("lengthSideB cannot be negative: " + lengthSideA);
      if (Math.abs(angleBetweenAAndB) > Math.PI)
         throw new IllegalArgumentException("angleBetweenAAndB " + angleBetweenAAndB + " does not define a triangle.");

      return EuclidCoreTools.squareRoot(lengthSideA * lengthSideA + lengthSideB * lengthSideB
                                        - 2.0 * lengthSideA * lengthSideB * EuclidCoreTools.cos(angleBetweenAAndB));
   }
}
