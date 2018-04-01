package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextPoint2D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextPoint3D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextQuaternion;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextVector2D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextVector2DWithFixedLength;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextVector3D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextVector3DWithFixedLength;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * This class provides random generators to generate random geometry objects.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public abstract class EuclidGeometryRandomTools
{
   /**
    * Generates a random line 2D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 2D.
    * @deprecated Use {@link #nextLine2D(Random)} instead
    */
   @Deprecated
   public static Line2D generateRandomLine2D(Random random)
   {
      return nextLine2D(random);
   }

   /**
    * Generates a random line 2D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 2D.
    */
   public static Line2D nextLine2D(Random random)
   {
      return new Line2D(nextPoint2D(random), nextVector2D(random));
   }

   /**
    * Generates a random line 2D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 2D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    * @deprecated Use {@link #nextLine2D(Random,double)} instead
    */
   @Deprecated
   public static Line2D generateRandomLine2D(Random random, double pointMinMax)
   {
      return nextLine2D(random, pointMinMax);
   }

   /**
    * Generates a random line 2D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 2D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Line2D nextLine2D(Random random, double pointMinMax)
   {
      return new Line2D(nextPoint2D(random, pointMinMax), nextVector2D(random));
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 3D.
    * @deprecated Use {@link #nextLine3D(Random)} instead
    */
   @Deprecated
   public static Line3D generateRandomLine3D(Random random)
   {
      return nextLine3D(random);
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 3D.
    */
   public static Line3D nextLine3D(Random random)
   {
      return new Line3D(nextPoint3D(random), nextVector3D(random));
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    * @deprecated Use {@link #nextLine3D(Random,double)} instead
    */
   @Deprecated
   public static Line3D generateRandomLine3D(Random random, double pointMinMax)
   {
      return nextLine3D(random, pointMinMax);
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Line3D nextLine3D(Random random, double pointMinMax)
   {
      return new Line3D(nextPoint3D(random, pointMinMax), nextVector3D(random));
   }

   /**
    * Generates a random line segment 1D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint} &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint} &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 1D.
    * @deprecated Use {@link #nextLineSegment1D(Random)} instead
    */
   @Deprecated
   public static LineSegment1D generateRandomLineSegment1D(Random random)
   {
      return nextLineSegment1D(random);
   }

   /**
    * Generates a random line segment 1D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint} &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint} &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 1D.
    */
   public static LineSegment1D nextLineSegment1D(Random random)
   {
      return new LineSegment1D(nextDouble(random), nextDouble(random));
   }

   /**
    * Generates a random line segment 1D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint} &in; [-minMax; minMax].
    * <li>{@code secondEndpoint} &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each line segment's endpoints.
    * @return the random line segment 1D.
    * @throws RuntimeException if {@code minMax < 0}.
    * @deprecated Use {@link #nextLineSegment1D(Random,double)} instead
    */
   @Deprecated
   public static LineSegment1D generateRandomLineSegment1D(Random random, double minMax)
   {
      return nextLineSegment1D(random, minMax);
   }

   /**
    * Generates a random line segment 1D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint} &in; [-minMax; minMax].
    * <li>{@code secondEndpoint} &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each line segment's endpoints.
    * @return the random line segment 1D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment1D nextLineSegment1D(Random random, double minMax)
   {
      return new LineSegment1D(nextDouble(random, minMax), nextDouble(random, minMax));
   }

   /**
    * Generates a random line segment 2D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 2D.
    * @deprecated Use {@link #nextLineSegment2D(Random)} instead
    */
   @Deprecated
   public static LineSegment2D generateRandomLineSegment2D(Random random)
   {
      return nextLineSegment2D(random);
   }

   /**
    * Generates a random line segment 2D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 2D.
    */
   public static LineSegment2D nextLineSegment2D(Random random)
   {
      return new LineSegment2D(nextPoint2D(random), nextPoint2D(random));
   }

   /**
    * Generates a random line segment 2D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 2D.
    * @throws RuntimeException if {@code minMax < 0}.
    * @deprecated Use {@link #nextLineSegment2D(Random,double)} instead
    */
   @Deprecated
   public static LineSegment2D generateRandomLineSegment2D(Random random, double minMax)
   {
      return nextLineSegment2D(random, minMax);
   }

   /**
    * Generates a random line segment 2D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 2D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment2D nextLineSegment2D(Random random, double minMax)
   {
      return new LineSegment2D(nextPoint2D(random, minMax), nextPoint2D(random, minMax));
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 3D.
    * @deprecated Use {@link #nextLineSegment3D(Random)} instead
    */
   @Deprecated
   public static LineSegment3D generateRandomLineSegment3D(Random random)
   {
      return nextLineSegment3D(random);
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 3D.
    */
   public static LineSegment3D nextLineSegment3D(Random random)
   {
      return new LineSegment3D(nextPoint3D(random), nextPoint3D(random));
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 3D.
    * @throws RuntimeException if {@code minMax < 0}.
    * @deprecated Use {@link #nextLineSegment3D(Random,double)} instead
    */
   @Deprecated
   public static LineSegment3D generateRandomLineSegment3D(Random random, double minMax)
   {
      return nextLineSegment3D(random, minMax);
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 3D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment3D nextLineSegment3D(Random random, double minMax)
   {
      return new LineSegment3D(nextPoint3D(random, minMax), nextPoint3D(random, minMax));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @return the random bounding box.
    * @deprecated Use {@link #nextBoundingBox2D(Random)} instead
    */
   @Deprecated
   public static BoundingBox2D generateRandomBoundingBox2D(Random random)
   {
      return nextBoundingBox2D(random);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @return the random bounding box.
    */
   public static BoundingBox2D nextBoundingBox2D(Random random)
   {
      return nextBoundingBox2D(random, 1.0, 1.0);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    * @deprecated Use {@link #nextBoundingBox2D(Random,double,double)} instead
    */
   @Deprecated
   public static BoundingBox2D generateRandomBoundingBox2D(Random random, double centerMinMax, double sizeMax)
   {
      return nextBoundingBox2D(random, centerMinMax, sizeMax);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static BoundingBox2D nextBoundingBox2D(Random random, double centerMinMax, double sizeMax)
   {
      Point2D center = nextPoint2D(random, centerMinMax);
      Vector2D halfSize = nextVector2D(random, 0.0, sizeMax / 2.0);
      return BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @return the random bounding box.
    * @deprecated Use {@link #nextBoundingBox3D(Random)} instead
    */
   @Deprecated
   public static BoundingBox3D generateRandomBoundingBox3D(Random random)
   {
      return nextBoundingBox3D(random);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @return the random bounding box.
    */
   public static BoundingBox3D nextBoundingBox3D(Random random)
   {
      return nextBoundingBox3D(random, 1.0, 1.0);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    * @deprecated Use {@link #nextBoundingBox3D(Random,double,double)} instead
    */
   @Deprecated
   public static BoundingBox3D generateRandomBoundingBox3D(Random random, double centerMinMax, double sizeMax)
   {
      return nextBoundingBox3D(random, centerMinMax, sizeMax);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static BoundingBox3D nextBoundingBox3D(Random random, double centerMinMax, double sizeMax)
   {
      Point3D center = nextPoint3D(random, centerMinMax);
      Vector3D halfSize = nextVector3D(random, 0.0, sizeMax / 2.0);
      return BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);
   }

   /**
    * Generates a random orientation 2D.
    * <p>
    * <ul>
    * <li>{@code yaw} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random orientation 2D.
    * @deprecated Use {@link #nextOrientation2D(Random)} instead
    */
   @Deprecated
   public static Orientation2D generateRandomOrientation2D(Random random)
   {
      return nextOrientation2D(random);
   }

   /**
    * Generates a random orientation 2D.
    * <p>
    * <ul>
    * <li>{@code yaw} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random orientation 2D.
    */
   public static Orientation2D nextOrientation2D(Random random)
   {
      return new Orientation2D(nextDouble(random, Math.PI));
   }

   /**
    * Generates a random orientation 2D.
    * <p>
    * <ul>
    * <li>{@code yaw} &in; [-{@code minMax}; {@code minMax}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value orientation 2D's angle.
    * @return the random orientation 2D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    * @deprecated Use {@link #nextOrientation2D(Random,double)} instead
    */
   @Deprecated
   public static Orientation2D generateRandomOrientation2D(Random random, double minMax)
   {
      return nextOrientation2D(random, minMax);
   }

   /**
    * Generates a random orientation 2D.
    * <p>
    * <ul>
    * <li>{@code yaw} &in; [-{@code minMax}; {@code minMax}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value orientation 2D's angle.
    * @return the random orientation 2D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Orientation2D nextOrientation2D(Random random, double minMax)
   {
      return new Orientation2D(nextDouble(random, minMax));
   }

   /**
    * Generates a random plane 3D from a random point and a random unit-vector.
    * <p>
    * Each coordinate of the random point are in [-1, 1].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random plane 3D.
    * @deprecated Use {@link #nextPlane3D(Random)} instead
    */
   @Deprecated
   public static Plane3D generateRandomPlane3D(Random random)
   {
      return nextPlane3D(random);
   }

   /**
    * Generates a random plane 3D from a random point and a random unit-vector.
    * <p>
    * Each coordinate of the random point are in [-1, 1].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random plane 3D.
    */
   public static Plane3D nextPlane3D(Random random)
   {
      return nextPlane3D(random, 1.0);
   }

   /**
    * Generates a random plane 3D from a random point and a random unit-vector.
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the random point.
    * @return the random plane 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    * @deprecated Use {@link #nextPlane3D(Random,double)} instead
    */
   @Deprecated
   public static Plane3D generateRandomPlane3D(Random random, double pointMinMax)
   {
      return nextPlane3D(random, pointMinMax);
   }

   /**
    * Generates a random plane 3D from a random point and a random unit-vector.
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the random point.
    * @return the random plane 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Plane3D nextPlane3D(Random random, double pointMinMax)
   {
      Point3D pointOnPlane = nextPoint3D(random, pointMinMax);
      Vector3D planeNormal = nextVector3DWithFixedLength(random, 1.0);
      return new Plane3D(pointOnPlane, planeNormal);
   }

   /**
    * Generates a random pose 2D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random pose 2D.
    * @deprecated Use {@link #nextPose2D(Random)} instead
    */
   @Deprecated
   public static Pose2D generateRandomPose2D(Random random)
   {
      return nextPose2D(random);
   }

   /**
    * Generates a random pose 2D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random pose 2D.
    */
   public static Pose2D nextPose2D(Random random)
   {
      return new Pose2D(nextPoint2D(random), nextOrientation2D(random));
   }

   /**
    * Generates a random pose 2D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].
    * <li>{@code orientation} &in; [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param positionMinMax the maximum absolute value for each coordinate of the line's point.
    * @param orientationMinMax the maximum absolute value orientation 2D's angle.
    * @return the random pose 2D.
    * @throws RuntimeException if {@code positionMinMax < 0} or {@code orientationMinMax < 0}.
    */
   public static Pose2D nextPose2D(Random random, double positionMinMax, double orientationMinMax)
   {
      return new Pose2D(nextPoint2D(random, positionMinMax), nextOrientation2D(random, orientationMinMax));
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random pose 3D.
    * @deprecated Use {@link #nextPose3D(Random)} instead
    */
   @Deprecated
   public static Pose3D generateRandomPose3D(Random random)
   {
      return nextPose3D(random);
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random pose 3D.
    */
   public static Pose3D nextPose3D(Random random)
   {
      return new Pose3D(nextPoint3D(random), nextQuaternion(random));
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>X</sub> &in; [-{@code maxAbsoluteX}; {@code maxAbsoluteX}].
    * <li>{@code position}<sub>Y</sub> &in; [-{@code maxAbsoluteY}; {@code maxAbsoluteY}].
    * <li>{@code position}<sub>Z</sub> &in; [-{@code maxAbsoluteZ}; {@code maxAbsoluteZ}].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    * 
    * @param random the random generator to use.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate of the position part of
    *           the pose 3D.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate of the position part of
    *           the pose 3D.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate of the position part of
    *           the pose 3D.
    * @return the random pose 3D.
    */
   public static Pose3D nextPose3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new Pose3D(nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ), nextQuaternion(random));
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].
    * <li>{@code orientation.getAngle()} &in; [-{@code orientationMinMax};
    * {@code orientationMinMax}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param positionMinMax the maximum absolute value for each coordinate of the pose's position.
    * @param orientationMinMax the maximum absolute value of the rotation angle for the pose's
    *           orientation.
    * @return the random pose 3D.
    * @throws RuntimeException if {@code positionMinMax < 0} or {@code orientationMinMax < 0}.
    */
   public static Pose3D nextPose3D(Random random, double positionMinMax, double orientationMinMax)
   {
      return new Pose3D(nextPoint3D(random, positionMinMax), nextQuaternion(random, orientationMinMax));
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices
    * and the size of the point cloud from which it is generated.
    *
    * @param random the random generator to use.
    * @param maxAbsoluteXY the maximum absolute value for each coordinate of the vertices.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for
    *           computing the random convex polygon. The size of the resulting convex polygon will
    *           be less than {@code numberOfPossiblePoints}.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static ConvexPolygon2D nextConvexPolygon2D(Random random, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.nextPointCloud2D(random, 0.0, maxAbsoluteXY, numberOfPossiblePoints);
      return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
   }

   /**
    * Generates a random 2D point cloud given a random average, range, and size.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the random average.
    * @param minMax the range of the point cloud in the three directions.
    * @param numberOfPoints the size of the point cloud to generate.
    * @return the random 2D point cloud.
    * @deprecated Use {@link #nextPointCloud2D(Random,double,double,int)} instead
    */
   @Deprecated
   public static List<Point2D> generateRandomPointCloud2D(Random random, double centerMinMax, double minMax, int numberOfPoints)
   {
      return nextPointCloud2D(random, centerMinMax, minMax, numberOfPoints);
   }

   /**
    * Generates a random 2D point cloud given a random average, range, and size.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the random average.
    * @param minMax the range of the point cloud in the three directions.
    * @param numberOfPoints the size of the point cloud to generate.
    * @return the random 2D point cloud.
    */
   public static List<Point2D> nextPointCloud2D(Random random, double centerMinMax, double minMax, int numberOfPoints)
   {
      List<Point2D> pointCloud2D = new ArrayList<>();

      Point2D center = nextPoint2D(random, centerMinMax);
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point2D point = nextPoint2D(random, minMax);
         point.add(center);
         pointCloud2D.add(point);
      }

      return pointCloud2D;
   }

   /**
    * Generates a random convex polygon 2D which construction is based on the generation of a random
    * circle onto which the vertices are generated.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength maximum distance between two successive vertices constraining the size of
    *           the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @return the random convex polygon 2D.
    * @deprecated Use {@link #nextCircleBasedConvexPolygon2D(Random,double,double,int)} instead
    */
   @Deprecated
   public static List<Point2D> generateRandomCircleBasedConvexPolygon2D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices)
   {
      return nextCircleBasedConvexPolygon2D(random, centerMinMax, maxEdgeLength, numberOfVertices);
   }

   /**
    * Generates a random convex polygon 2D which construction is based on the generation of a random
    * circle onto which the vertices are generated.
    *
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength maximum distance between two successive vertices constraining the size of
    *           the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @return the random convex polygon 2D.
    */
   public static List<Point2D> nextCircleBasedConvexPolygon2D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return Collections.emptyList();
      if (numberOfVertices == 1)
         return Collections.singletonList(nextPoint2D(random, centerMinMax));
      if (numberOfVertices == 2)
      {
         Vector2D halfEdgeLentgh = nextVector2DWithFixedLength(random, 0.5 * maxEdgeLength * random.nextDouble());
         Point2D center = nextPoint2D(random, centerMinMax);
         Point2D a = new Point2D();
         Point2D b = new Point2D();
         a.add(center, halfEdgeLentgh);
         b.sub(center, halfEdgeLentgh);
         List<Point2D> points = new ArrayList<>();
         points.add(a);
         points.add(b);
         return points;
      }

      // Generating random angles from vertex to vertex
      double[] dTheta = new double[numberOfVertices];
      double sum = 0.0;

      for (int i = 0; i < numberOfVertices; i++)
      {
         dTheta[i] = nextDouble(random, 0.001, 1.0);
         sum += dTheta[i];
      }
      // Adding the angle for the last segment
      sum += random.nextDouble();

      // Re-scaling the all the angles such that sum is equal to 2*pi
      double scale = 2.0 * Math.PI / sum;

      for (int i = 0; i < numberOfVertices; i++)
         dTheta[i] *= scale;

      // Generating the clockwise ordered vertices distributed on a circle centered at (0, 0).
      List<Point2D> clockwiseVertices = new ArrayList<>();
      double theta = 0.0;
      // Add a random yaw angle on all the vertices
      double yaw = nextDouble(random, Math.PI);
      clockwiseVertices.add(new Point2D(Math.cos(yaw), Math.sin(yaw)));

      for (int i = 1; i < numberOfVertices; i++)
      {
         theta -= dTheta[i];
         double x = Math.cos(theta + yaw);
         double y = Math.sin(theta + yaw);
         clockwiseVertices.add(new Point2D(x, y));
      }

      // Re-scaling the vertices such that the max edge length is contained in [0, maxEdgeLength]
      double currentMaxEdgeLength = 0.0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseVertices.get(i);
         Point2D nextVertex = clockwiseVertices.get((i + 1) % numberOfVertices);
         currentMaxEdgeLength = Math.max(currentMaxEdgeLength, vertex.distance(nextVertex));
      }

      // Limiting the min value from the random to prevent obtaining a polygon that is way too small
      scale = nextDouble(random, 0.1, 1.0) * maxEdgeLength / currentMaxEdgeLength;
      for (int i = 0; i < numberOfVertices; i++)
         clockwiseVertices.get(i).scale(scale);

      // By definition in this library, a convex polygon's first vertex is located has the min x coordinate.
      // If more than one vertex is located at min X, then it is the vertex with the max Y that is the first.
      int indexOfFirstVertex = 0;
      Point2D firstVertex = clockwiseVertices.get(0);

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D currentVertex = clockwiseVertices.get(i);
         if (firstVertex.getX() > currentVertex.getX())
         {
            firstVertex = currentVertex;
            indexOfFirstVertex = i;
         }
         else if (firstVertex.getX() == currentVertex.getX() && firstVertex.getY() < currentVertex.getY())
         {
            firstVertex = currentVertex;
            indexOfFirstVertex = i;
         }
      }

      // Make a new list with the vertices properly ordered and shifted to be around a random center
      Point2D center = nextPoint2D(random, centerMinMax);
      List<Point2D> convexPolygon2D = new ArrayList<>();

      for (int i = 0; i < numberOfVertices; i++)
      {
         int indexInOtherList = (i + indexOfFirstVertex) % numberOfVertices;
         Point2D vertex = clockwiseVertices.get(indexInOtherList);
         vertex.add(center);
         convexPolygon2D.add(vertex);
      }

      return convexPolygon2D;
   }

   /**
    * Generates a fixed-size supplier of random vertex 2D.
    * 
    * @param random the random generator to use.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static Vertex2DSupplier nextVertex2DSupplier(Random random, int numberOfVertices)
   {
      List<Point2D> vertices = IntStream.range(0, numberOfVertices).mapToObj(i -> nextPoint2D(random)).collect(Collectors.toList());
      return Vertex2DSupplier.asVertex2DSupplier(vertices);
   }

   /**
    * Generates a fixed-size supplier of random vertex 3D.
    * 
    * @param random the random generator to use.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static Vertex3DSupplier nextVertex3DSupplier(Random random, int numberOfVertices)
   {
      List<Point3D> vertices = IntStream.range(0, numberOfVertices).mapToObj(i -> nextPoint3D(random)).collect(Collectors.toList());
      return Vertex3DSupplier.asVertex3DSupplier(vertices);
   }
}
