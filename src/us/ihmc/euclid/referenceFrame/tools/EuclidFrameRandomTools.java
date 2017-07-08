package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

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
      return generateRandomReferenceFrame("randomFrame" + random.nextInt(), random, ReferenceFrame.getWorldFrame());
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
      return generateRandomReferenceFrame("randomFrame" + random.nextInt(), random, parentFrame);
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
      RigidBodyTransform transformFromParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
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
      return generateRandomReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), 20);
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
      return generateRandomReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), numberOfReferenceFrames);
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
      ReferenceFrame[] referenceFrames = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFrames[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i);
         ReferenceFrame parentFrame = referenceFrames[parentFrameIndex];
         referenceFrames[i + 1] = generateRandomReferenceFrame(frameNamePrefix + i, random, parentFrame);
      }

      return referenceFrames;
   }

   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame frame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePoint3D(frame, EuclidCoreRandomTools.generateRandomPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ));
   }

   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame frame, double minX, double maxX, double minY, double maxY, double minZ,
                                                       double maxZ)
   {
      return new FramePoint3D(frame, EuclidCoreRandomTools.generateRandomPoint3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame frame)
   {
      return new FrameVector3D(frame, EuclidCoreRandomTools.generateRandomVector3D(random));
   }

   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame frame, double minX, double maxX, double minY, double maxY, double minZ,
                                                         double maxZ)
   {
      return new FrameVector3D(frame, EuclidCoreRandomTools.generateRandomVector3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame zUpFrame, double minX, double maxX, double minY, double maxY)
   {
      return new FramePoint2D(zUpFrame, EuclidCoreRandomTools.generateRandomPoint2D(random, minX, maxX, minY, maxY));
   }

   public static FrameVector2D generateRandomFrameVector2d(Random random, ReferenceFrame zUpFrame)
   {
      return new FrameVector2D(zUpFrame, EuclidCoreRandomTools.generateRandomVector2D(random));
   }
}
