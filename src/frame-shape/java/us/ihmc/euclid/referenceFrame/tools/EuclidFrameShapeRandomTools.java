package us.ihmc.euclid.referenceFrame.tools;

import java.util.List;
import java.util.Random;

import us.ihmc.euclid.referenceFrame.FixedFrameShape3DPose;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePointShape3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.FrameShape3DPose;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameFace3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameVertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides random generators to generate random frame shapes.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameShapeRandomTools
{
   private EuclidFrameShapeRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Generates a random pose using {@link EuclidShapeRandomTools#nextShape3DPose(Random)}.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @return the random pose.
    */
   public static FrameShape3DPose nextFrameShape3DPose(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameShape3DPose(referenceFrame, EuclidShapeRandomTools.nextShape3DPose(random));
   }

   /**
    * Generates a random pose using {@link EuclidShapeRandomTools#nextShape3DPose(Random)}.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @return the random pose.
    */
   public static FixedFrameShape3DPose nextFixedFrameShape3DPose(Random random, ReferenceFrame referenceFrame)
   {
      return new FixedFrameShape3DPose((ReferenceFrameHolder) () -> referenceFrame, EuclidShapeRandomTools.nextShape3DPose(random));
   }

   /**
    * Generates a random box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random box 3D.
    */
   public static FrameBox3D nextFrameBox3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameBox3D(referenceFrame, EuclidShapeRandomTools.nextBox3D(random));
   }

   /**
    * Generates a random box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minSize        the minimum value for each component of the box size.
    * @param maxSize        the maximum value for each component of the box size.
    * @return the random box 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static FrameBox3D nextFrameBox3D(Random random, ReferenceFrame referenceFrame, double minSize, double maxSize)
   {
      return new FrameBox3D(referenceFrame, EuclidShapeRandomTools.nextBox3D(random, minSize, maxSize));
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code length} &in; [0.0; 1.0].
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random capsule 3D.
    */
   public static FrameCapsule3D nextFrameCapsule3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameCapsule3D(referenceFrame, EuclidShapeRandomTools.nextCapsule3D(random));
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code length} &in; [{@code minLength}; {@code maxLength}].
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minLength      the minimum value for the length.
    * @param maxLength      the maximum value for the length.
    * @param minRadius      the minimum value for the radius.
    * @param maxRadius      the maximum value for the radius.
    * @return the random capsule 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static FrameCapsule3D nextFrameCapsule3D(Random random,
                                                   ReferenceFrame referenceFrame,
                                                   double minLength,
                                                   double maxLength,
                                                   double minRadius,
                                                   double maxRadius)
   {
      return new FrameCapsule3D(referenceFrame, EuclidShapeRandomTools.nextCapsule3D(random, minLength, maxLength, minRadius, maxRadius));
   }

   /**
    * Generates a random cylinder 3D.
    * <ul>
    * <li>{@code length} &in; [0.0; 1.0].
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random cylinder 3D.
    */
   public static FrameCylinder3D nextFrameCylinder3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameCylinder3D(referenceFrame, EuclidShapeRandomTools.nextCylinder3D(random));
   }

   /**
    * Generates a random cylinder 3D.
    * <ul>
    * <li>{@code length} &in; [{@code minLength}; {@code maxLength}].
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minLength      the minimum value for the length.
    * @param maxLength      the maximum value for the length.
    * @param minRadius      the minimum value for the radius.
    * @param maxRadius      the maximum value for the radius.
    * @return the random cylinder 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static FrameCylinder3D nextFrameCylinder3D(Random random,
                                                     ReferenceFrame referenceFrame,
                                                     double minLength,
                                                     double maxLength,
                                                     double minRadius,
                                                     double maxRadius)
   {
      return new FrameCylinder3D(referenceFrame, EuclidShapeRandomTools.nextCylinder3D(random, minLength, maxLength, minRadius, maxRadius));
   }

   /**
    * Generates a random ellipsoid 3D.
    * <ul>
    * <li>{@code radii}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random radii 3D.
    */
   public static FrameEllipsoid3D nextFrameEllipsoid3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameEllipsoid3D(referenceFrame, EuclidShapeRandomTools.nextEllipsoid3D(random));
   }

   /**
    * Generates a random ellipsoid 3D.
    * <ul>
    * <li>{@code radii}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minRadius      the minimum value for each component of the ellipsoid radius.
    * @param maxRadius      the maximum value for each component of the ellipsoid radius.
    * @return the random ellipsoid 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static FrameEllipsoid3D nextFrameEllipsoid3D(Random random, ReferenceFrame referenceFrame, double minRadius, double maxRadius)
   {
      return new FrameEllipsoid3D(referenceFrame, EuclidShapeRandomTools.nextEllipsoid3D(random, minRadius, maxRadius));
   }

   /**
    * Generates a random point shape.
    * <p>
    * {@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random point shape.
    */
   public static FramePointShape3D nextFramePointShape3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePointShape3D(EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame));
   }

   /**
    * Generates a random point.
    * <ul>
    * <li>{@code position.x} &in; [-{@code minMax}; {@code minMax}].
    * <li>{@code position.y} &in; [-{@code minMax}; {@code minMax}].
    * <li>{@code position.z} &in; [-{@code minMax}; {@code minMax}].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minMax         the maximum absolute value for each coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePointShape3D nextFramePointShape3D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePointShape3D(EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame, minMax));
   }

   /**
    * Generates a random ramp 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random ramp 3D.
    */
   public static FrameRamp3D nextFrameRamp3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameRamp3D(referenceFrame, EuclidShapeRandomTools.nextRamp3D(random));
   }

   /**
    * Generates a random ramp 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minSize        the minimum value for each component of the ramp size.
    * @param maxSize        the maximum value for each component of the ramp size.
    * @return the random ramp 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static FrameRamp3D nextFrameRamp3D(Random random, ReferenceFrame referenceFrame, double minSize, double maxSize)
   {
      return new FrameRamp3D(referenceFrame, EuclidShapeRandomTools.nextRamp3D(random, minSize, maxSize));
   }

   /**
    * Generates a random sphere 3D.
    * <ul>
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random sphere 3D.
    */
   public static FrameSphere3D nextFrameSphere3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameSphere3D(referenceFrame, EuclidShapeRandomTools.nextSphere3D(random));
   }

   /**
    * Generates a random sphere 3D.
    * <ul>
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param minRadius      the minimum value for the radius.
    * @param maxRadius      the maximum value for the radius.
    * @return the random sphere 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static FrameSphere3D nextFrameSphere3D(Random random, ReferenceFrame referenceFrame, double minRadius, double maxRadius)
   {
      return new FrameSphere3D(referenceFrame, EuclidShapeRandomTools.nextSphere3D(random, minRadius, maxRadius));
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame face's reference frame.
    * @return the random face.
    */
   public static FrameFace3D nextCircleBasedFrameFace3D(Random random, ReferenceFrame referenceFrame)
   {
      return nextCircleBasedFrameFace3D(random, referenceFrame, 5.0);
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame face's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinates of the circle's center.
    * @return the random face.
    */
   public static FrameFace3D nextCircleBasedFrameFace3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return nextCircleBasedFrameFace3D(random, referenceFrame, centerMinMax, 1.0, 15);
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random           the random generator to use.
    * @param referenceFrame   the random frame face's reference frame.
    * @param centerMinMax     the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength    maximum distance between two successive vertices constraining the size of
    *                         the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @return the random face.
    */
   public static FrameFace3D nextCircleBasedFrameFace3D(Random random,
                                                        ReferenceFrame referenceFrame,
                                                        double centerMinMax,
                                                        double maxEdgeLength,
                                                        int numberOfVertices)
   {
      return nextCircleBasedFrameFace3D(random,
                                        referenceFrame,
                                        centerMinMax,
                                        maxEdgeLength,
                                        numberOfVertices,
                                        EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random           the random generator to use.
    * @param referenceFrame   the random frame face's reference frame.
    * @param centerMinMax     the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength    maximum distance between two successive vertices constraining the size of
    *                         the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @param faceNormal       the normal of the face and the axis of revolution of the circle. Not
    *                         modified.
    * @return the random face.
    */
   public static FrameFace3D nextCircleBasedFrameFace3D(Random random,
                                                        ReferenceFrame referenceFrame,
                                                        double centerMinMax,
                                                        double maxEdgeLength,
                                                        int numberOfVertices,
                                                        Vector3DReadOnly faceNormal)
   {
      List<Point3D> vertices = EuclidShapeRandomTools.nextCircleBasedConvexPolygon3D(random, centerMinMax, maxEdgeLength, numberOfVertices, faceNormal);
      FrameFace3D face3D = new FrameFace3D(() -> referenceFrame, faceNormal);
      vertices.forEach(vertex -> face3D.addVertex(new FrameVertex3D(face3D, vertex)));
      return face3D;
   }

   /**
    * Generates a random convex polytope 3D.
    * <p>
    * The convex polytope is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link EuclidShapeRandomTools#nextConeConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextCubeConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextCylinderConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextIcosahedronBasedConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextIcoSphereBasedConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextPointCloudBasedConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextPyramidConvexPolytope3D(Random)}.
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextConvexPolytope3D(random));
   }

   /**
    * Generates a random convex polytope 3D in the same way as
    * {@link #nextConeFrameConvexPolytope3D(Random, ReferenceFrame)} and in addition allows the
    * generation of:
    * <ul>
    * <li>empty convex polytope.
    * <li>convex polytope with a single vertex.
    * <li>convex polytope with a single edge.
    * <li>convex polytope with a single face.
    * </ul>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextFrameConvexPolytope3DWithEdgeCases(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random));
   }

   /**
    * Generates a convex polytope by discretizing a randomly generated cone 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextConeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextConeConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope by discretizing a randomly generated cone 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the cone's base center.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextConeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextConeConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope by discretizing a randomly generated cone 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the cone's base center.
    * @param heightMin      the minimum value for the height of the cone.
    * @param heightMax      the maximum value for the height of the cone.
    * @param radiusMin      the minimum value for the radius of the cone base.
    * @param radiusMax      the maximum value for the radius of the cone base.
    * @param divisionsMin   the minimum number of divisions for discretizing the cone.
    * @param divisionsMax   the maximum number of divisions for discretizing the cone.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextConeFrameConvexPolytope3D(Random random,
                                                                     ReferenceFrame referenceFrame,
                                                                     double centerMinMax,
                                                                     double heightMin,
                                                                     double heightMax,
                                                                     double radiusMin,
                                                                     double radiusMax,
                                                                     int divisionsMin,
                                                                     int divisionsMax)
   {
      return new FrameConvexPolytope3D(referenceFrame,
                                       EuclidShapeRandomTools.nextConeConvexPolytope3D(random,
                                                                                       centerMinMax,
                                                                                       heightMin,
                                                                                       heightMax,
                                                                                       radiusMin,
                                                                                       radiusMax,
                                                                                       divisionsMin,
                                                                                       divisionsMax));
   }

   /**
    * Generates a convex polytope from a randomly generated cube 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextCubeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextCubeConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a randomly generated cube 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextCubeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextCubeConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a randomly generated cube 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param edgeLengthMin  the minimum value for the cube's edge length.
    * @param edgeLengthMax  the maximum value for the cube's edge length.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextCubeFrameConvexPolytope3D(Random random,
                                                                     ReferenceFrame referenceFrame,
                                                                     double centerMinMax,
                                                                     double edgeLengthMin,
                                                                     double edgeLengthMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextCubeConvexPolytope3D(random, centerMinMax, edgeLengthMin, edgeLengthMax));
   }

   /**
    * Generates a convex polytope from a randomly generated cylinder 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextCylinderFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextCylinderConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a randomly generated cylinder 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextCylinderFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextCylinderConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a randomly generated cylinder 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param lengthMin      the minimum value for the length of the cylinder.
    * @param lengthMax      the maximum value for the length of the cylinder.
    * @param radiusMin      the minimum value for the radius of the cylinder.
    * @param radiusMax      the maximum value for the radius of the cylinder.
    * @param divisionsMin   the minimum number of divisions for discretizing the cylinder.
    * @param divisionsMax   the maximum number of divisions for discretizing the cylinder.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextCylinderFrameConvexPolytope3D(Random random,
                                                                         ReferenceFrame referenceFrame,
                                                                         double centerMinMax,
                                                                         double lengthMin,
                                                                         double lengthMax,
                                                                         double radiusMin,
                                                                         double radiusMax,
                                                                         int divisionsMin,
                                                                         int divisionsMax)
   {
      return new FrameConvexPolytope3D(referenceFrame,
                                       EuclidShapeRandomTools.nextCylinderConvexPolytope3D(random,
                                                                                           centerMinMax,
                                                                                           lengthMin,
                                                                                           lengthMax,
                                                                                           radiusMin,
                                                                                           radiusMax,
                                                                                           divisionsMin,
                                                                                           divisionsMax));
   }

   /**
    * Generates a convex polytope from a randomly generated icosahedron 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextIcosahedronBasedFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextIcosahedronBasedConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a randomly generated icosahedron 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextIcosahedronBasedFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextIcosahedronBasedConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a randomly generated icosahedron 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param radiusMin      the minimum value for the radius of the circumscribed sphere of the
    *                       icosahedron.
    * @param radiusMax      the maximum value for the radius of the circumscribed sphere of the
    *                       icosahedron.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextIcosahedronBasedFrameConvexPolytope3D(Random random,
                                                                                 ReferenceFrame referenceFrame,
                                                                                 double centerMinMax,
                                                                                 double radiusMin,
                                                                                 double radiusMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextIcosahedronBasedConvexPolytope3D(random, centerMinMax, radiusMin, radiusMax));
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static FrameConvexPolytope3D nextIcoSphereBasedFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static FrameConvexPolytope3D nextIcoSphereBasedFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param radiusMin      the minimum value for the radius of the ico-sphere.
    * @param radiusMax      the maximum value for the radius of the ico-sphere.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static FrameConvexPolytope3D nextIcoSphereBasedFrameConvexPolytope3D(Random random,
                                                                               ReferenceFrame referenceFrame,
                                                                               double centerMinMax,
                                                                               double radiusMin,
                                                                               double radiusMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, radiusMin, radiusMax));
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param recursionLevel the recursion level that defines the resolution of the ico-sphere.
    * @param radiusMin      the minimum value for the radius of the ico-sphere.
    * @param radiusMax      the maximum value for the radius of the ico-sphere.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static FrameConvexPolytope3D nextIcoSphereBasedFrameConvexPolytope3D(Random random,
                                                                               ReferenceFrame referenceFrame,
                                                                               double centerMinMax,
                                                                               int recursionLevel,
                                                                               double radiusMin,
                                                                               double radiusMax)
   {
      return new FrameConvexPolytope3D(referenceFrame,
                                       EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, recursionLevel, radiusMin, radiusMax));
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPointCloudBasedFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextPointCloudBasedConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPointCloudBasedFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextPointCloudBasedConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param minMax         the range of the point cloud in the three directions.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPointCloudBasedFrameConvexPolytope3D(Random random,
                                                                                ReferenceFrame referenceFrame,
                                                                                double centerMinMax,
                                                                                double minMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextPointCloudBasedConvexPolytope3D(random, centerMinMax, minMax));
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random                 the random generator to use.
    * @param referenceFrame         the random frame convex polytope's reference frame.
    * @param centerMinMax           the maximum absolute value for each coordinate for the convex
    *                               polytope's centroid.
    * @param minMax                 the range of the point cloud in the three directions.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for computing
    *                               the random convex polytope. The size of the resulting convex
    *                               polytope will be less than {@code numberOfPossiblePoints}.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPointCloudBasedFrameConvexPolytope3D(Random random,
                                                                                ReferenceFrame referenceFrame,
                                                                                double centerMinMax,
                                                                                double minMax,
                                                                                int numberOfPossiblePoints)
   {
      return new FrameConvexPolytope3D(referenceFrame,
                                       EuclidShapeRandomTools.nextPointCloudBasedConvexPolytope3D(random, centerMinMax, minMax, numberOfPossiblePoints));
   }

   /**
    * Generates a convex polytope from a randomly generated pyramid.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPyramidFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextPyramidConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a randomly generated pyramid.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate of the pyramid's base
    *                       center.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPyramidFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextPyramidConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a randomly generated pyramid.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate of the pyramid's base
    *                       center.
    * @param heightMin      the minimum value for the height of the pyramid.
    * @param heightMax      the maximum value for the height of the pyramid.
    * @param baseLengthMin  the minimum value for the length of the pyramid's base.
    * @param baseLengthMax  the maximum value for the length of the pyramid's base.
    * @param baseWidthMin   the minimum value for the width of the pyramid's base.
    * @param baseWidthMax   the maximum value for the width of the pyramid's base.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextPyramidFrameConvexPolytope3D(Random random,
                                                                        ReferenceFrame referenceFrame,
                                                                        double centerMinMax,
                                                                        double heightMin,
                                                                        double heightMax,
                                                                        double baseLengthMin,
                                                                        double baseLengthMax,
                                                                        double baseWidthMin,
                                                                        double baseWidthMax)
   {
      return new FrameConvexPolytope3D(referenceFrame,
                                       EuclidShapeRandomTools.nextPyramidConvexPolytope3D(random,
                                                                                          centerMinMax,
                                                                                          heightMin,
                                                                                          heightMax,
                                                                                          baseLengthMin,
                                                                                          baseLengthMax,
                                                                                          baseWidthMin,
                                                                                          baseWidthMax));
   }

   /**
    * Generates a convex polytope from a single randomly generated edge.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextSingleEdgeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextSingleEdgeConvexPolytope3D(random));
   }

   /**
    * Generates a convex polytope from a single randomly generated edge.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate of the edge's center.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextSingleEdgeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextSingleEdgeConvexPolytope3D(random, centerMinMax));
   }

   /**
    * Generates a convex polytope from a single randomly generated edge.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate of the edge's center.
    * @param minMax         the range of the edge in the three directions.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextSingleEdgeFrameConvexPolytope3D(Random random, ReferenceFrame referenceFrame, double centerMinMax, double minMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextSingleEdgeConvexPolytope3D(random, centerMinMax, minMax));
   }

   /**
    * Generates a convex polytope from a randomly generated tetrahedron that contains the given point.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param point          the point the tetrahedron must contain. Not modified.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextFrameTetrahedronContainingPoint3D(Random random, ReferenceFrame referenceFrame, Point3DReadOnly point)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point));
   }

   /**
    * Generates a convex polytope from a randomly generated tetrahedron that contains the given point.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame convex polytope's reference frame.
    * @param point          the point the tetrahedron must contain. Not modified.
    * @param minMax         the range of the tetrahedron in the three directions.
    * @return the random convex polytope 3D.
    */
   public static FrameConvexPolytope3D nextFrameTetrahedronContainingPoint3D(Random random, ReferenceFrame referenceFrame, Point3DReadOnly point, double minMax)
   {
      return new FrameConvexPolytope3D(referenceFrame, EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point, minMax));
   }

   /**
    * Generates a random shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextFrameBox3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameCapsule3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameConvexPolytope3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameCylinder3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameEllipsoid3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFramePointShape3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameRamp3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameSphere3D(Random, ReferenceFrame)}.
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random shape 3D.
    */
   public static FrameShape3DBasics nextFrameShape3D(Random random, ReferenceFrame referenceFrame)
   {
      switch (random.nextInt(8))
      {
         case 0:
            return nextFrameBox3D(random, referenceFrame);
         case 1:
            return nextFrameCapsule3D(random, referenceFrame);
         case 2:
            return nextFrameConvexPolytope3D(random, referenceFrame);
         case 3:
            return nextFrameCylinder3D(random, referenceFrame);
         case 4:
            return nextFrameEllipsoid3D(random, referenceFrame);
         case 5:
            return nextFramePointShape3D(random, referenceFrame);
         case 6:
            return nextFrameRamp3D(random, referenceFrame);
         case 7:
            return nextFrameSphere3D(random, referenceFrame);
         default:
            throw new RuntimeException("Unexpected state.");
      }
   }

   /**
    * Generates a random convex shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextFrameBox3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameCapsule3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameConvexPolytope3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameCylinder3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameEllipsoid3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFramePointShape3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameRamp3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameSphere3D(Random, ReferenceFrame)}.
    * </ul>
    * </p>
    * <p>
    * This random generator excludes the possibility of generating concave shapes.
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @return the random convex shape 3D.
    */
   public static FrameShape3DBasics nextFrameConvexShape3D(Random random, ReferenceFrame referenceFrame)
   {
      switch (random.nextInt(8))
      {
         case 0:
            return nextFrameBox3D(random, referenceFrame);
         case 1:
            return nextFrameCapsule3D(random, referenceFrame);
         case 2:
            return nextFrameConvexPolytope3D(random, referenceFrame);
         case 3:
            return nextFrameCylinder3D(random, referenceFrame);
         case 4:
            return nextFrameEllipsoid3D(random, referenceFrame);
         case 5:
            return nextFramePointShape3D(random, referenceFrame);
         case 6:
            return nextFrameRamp3D(random, referenceFrame);
         case 7:
            return nextFrameSphere3D(random, referenceFrame);
         default:
            throw new RuntimeException("Unexpected state.");
      }
   }

   /**
    * Generates a random convex shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link EuclidShapeRandomTools#nextBox3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextCapsule3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextConvexPolytope3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextCylinder3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextEllipsoid3D(Random)}.
    * <li>In case of {@link PointShape3D}, the resulting shape is not random because fully constrained
    * by the given position.
    * <li>{@link EuclidShapeRandomTools#nextRamp3D(Random)}.
    * <li>{@link EuclidShapeRandomTools#nextSphere3D(Random)}.
    * </ul>
    * </p>
    * <p>
    * This random generator excludes the possibility of generating concave shapes.
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame shape's reference frame.
    * @param shapeCentroid  the position of the shape's centroid. Not modified.
    * @return the random convex shape 3D.
    */
   public static FrameShape3DBasics nextFrameConvexShape3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly shapeCentroid)
   {
      Shape3DBasics next = EuclidShapeRandomTools.nextConvexShape3D(random, shapeCentroid);

      if (next instanceof Box3DReadOnly)
         return new FrameBox3D(referenceFrame, (Box3DReadOnly) next);
      if (next instanceof Capsule3DReadOnly)
         return new FrameCapsule3D(referenceFrame, (Capsule3DReadOnly) next);
      if (next instanceof ConvexPolytope3DReadOnly)
         return new FrameConvexPolytope3D(referenceFrame, (ConvexPolytope3DReadOnly) next);
      if (next instanceof Cylinder3DReadOnly)
         return new FrameCylinder3D(referenceFrame, (Cylinder3DReadOnly) next);
      if (next instanceof Ellipsoid3DReadOnly)
         return new FrameEllipsoid3D(referenceFrame, (Ellipsoid3DReadOnly) next);
      if (next instanceof PointShape3DReadOnly)
         return new FramePointShape3D(referenceFrame, shapeCentroid);
      if (next instanceof Ramp3DReadOnly)
         return new FrameRamp3D(referenceFrame, (Ramp3DReadOnly) next);
      if (next instanceof Sphere3DReadOnly)
         return new FrameSphere3D(referenceFrame, (Sphere3DReadOnly) next);
      throw new RuntimeException("Unexpected state.");
   }
}
