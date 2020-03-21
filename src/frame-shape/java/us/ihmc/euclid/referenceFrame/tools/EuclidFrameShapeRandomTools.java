package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class EuclidFrameShapeRandomTools
{
   private EuclidFrameShapeRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
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
   public static FrameCapsule3D nextFrameCapsule3D(Random random, ReferenceFrame referenceFrame, double minLength, double maxLength, double minRadius,
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
   public static FrameCylinder3D nextFrameCylinder3D(Random random, ReferenceFrame referenceFrame, double minLength, double maxLength, double minRadius,
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
    * Generates a random shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextFrameBox3D(Random, ReferenceFrame)}.
    * <li>{@link #nextFrameCapsule3D(Random, ReferenceFrame)}.
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
      switch (random.nextInt(7))
      {
         case 0:
            return nextFrameBox3D(random, referenceFrame);
         case 1:
            return nextFrameCapsule3D(random, referenceFrame);
         case 2:
            return nextFrameCylinder3D(random, referenceFrame);
         case 3:
            return nextFrameEllipsoid3D(random, referenceFrame);
         case 4:
            return nextFramePointShape3D(random, referenceFrame);
         case 5:
            return nextFrameRamp3D(random, referenceFrame);
         case 6:
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
      switch (random.nextInt(7))
      {
         case 0:
            return nextFrameBox3D(random, referenceFrame);
         case 1:
            return nextFrameCapsule3D(random, referenceFrame);
         case 2:
            return nextFrameCylinder3D(random, referenceFrame);
         case 3:
            return nextFrameEllipsoid3D(random, referenceFrame);
         case 4:
            return nextFramePointShape3D(random, referenceFrame);
         case 5:
            return nextFrameRamp3D(random, referenceFrame);
         case 6:
            return nextFrameSphere3D(random, referenceFrame);
         default:
            throw new RuntimeException("Unexpected state.");
      }
   }
}
