package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public class EuclidFrameRandomTools
{
   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(Random random)
   {
      return generateRandomReferenceFrame(random, false);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(Random random, boolean use2DTransform)
   {
      return generateRandomReferenceFrame(random, ReferenceFrame.getWorldFrame(), use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(Random random, ReferenceFrame parentFrame)
   {
      return generateRandomReferenceFrame(random, parentFrame, false);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      return generateRandomReferenceFrame("randomFrame" + random.nextInt(), random, parentFrame, use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      return generateRandomReferenceFrame(frameName, random, parentFrame, false);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      RigidBodyTransform transformFromParent;
      if (use2DTransform)
         transformFromParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform2D(random);
      else
         transformFromParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      return ReferenceFrame.constructFrameWithUnchangingTransformFromParent(frameName, parentFrame, transformFromParent);
   }

   /**
    * Creates a tree structure of 20 random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random)
   {
      return generateRandomReferenceFrameTree(random, false);
   }

   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, boolean use2DTransforms)
   {
      return generateRandomReferenceFrameTree(random, 20, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, int numberOfReferenceFrames)
   {
      return generateRandomReferenceFrameTree(random, numberOfReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, int numberOfReferenceFrames, boolean use2DTransforms)
   {
      return generateRandomReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), numberOfReferenceFrames, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    * 
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and {@code rootFrame} at the first
    *         index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames)
   {
      return generateRandomReferenceFrameTree(frameNamePrefix, random, rootFrame, numberOfReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    * 
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and {@code rootFrame} at the first
    *         index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames,
                                                                   boolean use2DTransforms)
   {
      ReferenceFrame[] referenceFrames = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFrames[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i + 1);
         ReferenceFrame parentFrame = referenceFrames[parentFrameIndex];
         referenceFrames[i + 1] = generateRandomReferenceFrame(frameNamePrefix + i, random, parentFrame, use2DTransforms);
      }

      return referenceFrames;
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    */
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint3D(random));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-minMax; minMax]. <br>
    * {@code framePoint.y} &in; [-minMax; minMax]. <br>
    * {@code framePoint.z} &in; [-minMax; minMax]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint3D(random, minMax));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * {@code framePoint.z} &in; [min; max]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min the minimum value for each coordinate.
    * @param max the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint3D(random, min, max));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-maxAbsoluteX; maxAbsoluteX]. <br>
    * {@code framePoint.y} &in; [-maxAbsoluteY; maxAbsoluteY]. <br>
    * {@code framePoint.z} &in; [-maxAbsoluteZ; maxAbsoluteZ]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxAbsoluteX < 0}, {@code maxAbsoluteY < 0},
    *            {@code maxAbsoluteZ < 0}.
    */
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY,
                                                         double maxAbsoluteZ)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * {@code framePoint.z} &in; [minZ; maxZ]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX the minimum value for the x-coordinate.
    * @param maxX the maximum value for the x-coordinate.
    * @param minY the minimum value for the y-coordinate.
    * @param maxY the maximum value for the y-coordinate.
    * @param minZ the minimum value for the z-coordinate.
    * @param maxZ the maximum value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY,
                                                         double minZ, double maxZ)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random frame vector.
    */
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomVector3D(random));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>;
    * {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minMax tuple used to bound the maximum absolute value of each component of the
    *           generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly minMax)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomVector3D(random, minMax));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min tuple used as upper-bound for each component of the generated frame vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated frame vector. Not
    *           modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomVector3D(random, min, max));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min upper-bound for each component of the generated frame vector. Not modified.
    * @param max lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomVector3D(random, min, max));
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code frameVector.x} &in; [minX; maxX]. <br>
    * {@code frameVector.y} &in; [minY; maxY]. <br>
    * {@code frameVector.z} &in; [minZ; maxZ]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minX the minimum value for the x-component.
    * @param maxX the maximum value for the x-component.
    * @param minY the minimum value for the y-component.
    * @param maxY the maximum value for the y-component.
    * @param minZ the minimum value for the z-component.
    * @param maxZ the maximum value for the z-component.
    * @return the random vector.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY,
                                                           double minZ, double maxZ)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomVector3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param length the length of the generated frame vector.
    * @return the random frame vector.
    */
   public static FrameVector3D generateRandomFrameVector3DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, length));
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param vectorToBeOrthogonalTo the frame vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    */
   public static FrameVector3D generateRandomOrthogonalFrameVector3D(Random random, FrameVector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      return generateRandomOrthogonalFrameVector3D(random, vectorToBeOrthogonalTo.getReferenceFrame(), vectorToBeOrthogonalTo, normalize);
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    */
   public static FrameVector3D generateRandomOrthogonalFrameVector3D(Random random, ReferenceFrame referenceFrame, Vector3DReadOnly vectorToBeOrthogonalTo,
                                                                     boolean normalize)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, vectorToBeOrthogonalTo, normalize));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    */
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint2D(random));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-minMax; minMax].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint2D(random, minMax));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param min the minimum value for each coordinate.
    * @param max the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint2D(random, min, max));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX the minimum value for the x-coordinate.
    * @param maxX the maximum value for the x-coordinate.
    * @param minY the minimum value for the y-coordinate.
    * @param maxY the maximum value for the y-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minX > maxX} or {@code minY > maxY}.
    */
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.generateRandomPoint2D(random, minX, maxX, minY, maxY));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame vector.
    */
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.generateRandomVector2D(random));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min upper-bound for each component of the generated frame vector. Not modified.
    * @param max lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.generateRandomVector2D(random, min, min));
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param length the length of the generated frame vector.
    * @return the random frame vector.
    */
   public static FrameVector2D generateRandomFrameVector2DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, length));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>;
    * {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax tuple used to bound the maximum absolute value of each component of the
    *           generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly minMax)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.generateRandomVector2D(random, minMax));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min tuple used as upper-bound for each component of the generated frame vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated frame vector. Not
    *           modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.generateRandomVector2D(random, min, max));
   }

   /**
    * Generates a random frame quaternion.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @return the random frame quaternion.
    */
   public static FrameQuaternion generateRandomFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameQuaternion(referenceFrame, EuclidCoreRandomTools.generateRandomQuaternion(random));
   }
}
