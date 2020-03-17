package us.ihmc.euclid.referenceFrame.api;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;
import java.util.Map.Entry;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * This class provides tools that using reflection can perform a variety of comparison-based
 * assertions on a frame geometry given its corresponding frameless type.
 * <p>
 * These tools are still experimental and are improved through heavy internal usage for building
 * Euclid's test suite. The objective it to make this class usable for third party classes.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameAPITester
{
   private static final String READ_ONLY = "ReadOnly";
   private static final String BASICS = "Basics";
   private static final String FRAME = "Frame";
   private static final String FIXED_FRAME = "FixedFrame";

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean DEBUG = false;
   private final static int FRAME_CHECK_ITERATIONS = 10;
   private final static int FUNCTIONALITY_ITERATIONS = 50;
   private final static Random random = new Random(345345);
   private final static double epsilon = 1.0e-12;

   private final static Set<Class<?>> framelessTypesWithoutFrameEquivalent = new HashSet<>();
   private final static Map<Class<?>, Class<?>> framelessTypesToFrameTypesTable = new HashMap<>();
   private final static Map<Class<?>, RandomFrameTypeBuilder<?>> frameTypeBuilders = new HashMap<>();
   private final static Map<Class<?>, GenericTypeBuilder> framelessTypeBuilders = new HashMap<>();
   private final static Set<Class<?>> frameReadOnlyTypes = new HashSet<>();
   private final static Set<Class<?>> fixedFrameMutableTypes = new HashSet<>();
   private final static Set<Class<?>> mutableFrameMutableTypes = new HashSet<>();

   private final static Set<Class<?>> frameRandomGeneratorLibrary = new HashSet<>();
   private final static Set<Class<?>> framelessRandomGeneratorLibrary = new HashSet<>();

   static
   {
      registerEuclidFrameTypes();
   }

   private static void registerEuclidFrameTypes()
   {
      registerFrameRandomGeneratorClasses(EuclidFrameRandomTools.class);
      registerFramelessRandomGeneratorClasses(EuclidCoreRandomTools.class, EuclidGeometryRandomTools.class);

      registerFrameTypesSmart(FrameTuple2DBasics.class, FrameVector2DBasics.class, FramePoint2DBasics.class);
      registerFrameTypesSmart(FrameTuple3DBasics.class, FrameVector3DBasics.class, FramePoint3DBasics.class);
      registerFrameTypesSmart(FrameTuple4DBasics.class, FrameVector4DBasics.class, FrameQuaternionBasics.class);
      registerFrameTypeSmart(FrameYawPitchRollBasics.class);
      registerFrameTypesSmart(FrameRotationMatrixBasics.class, FrameMatrix3DBasics.class);
      registerFrameTypesSmart(FrameOrientation2DBasics.class, FrameOrientation3DBasics.class);
      registerFrameTypesSmart(FramePose2DBasics.class, FramePose3DBasics.class);
      registerFrameTypesSmart(FrameLine2DBasics.class, FrameLine3DBasics.class);
      registerFrameTypesSmart(FrameLineSegment2DBasics.class, FrameLineSegment3DBasics.class);
      registerFrameTypesSmart(FrameConvexPolygon2DBasics.class);
      registerFrameTypesSmart(FrameBoundingBox2DBasics.class, FrameBoundingBox3DBasics.class);

      registerReadOnlyFrameTypeSmart(FrameVertex2DSupplier.class);
      registerReadOnlyFrameTypeSmart(FrameVertex3DSupplier.class);

      registerFramelessTypeSmart(AxisAngleBasics.class);
      registerFramelessType(RotationScaleMatrix.class, RotationScaleMatrixReadOnly.class, EuclidCoreRandomTools::nextRotationScaleMatrix);
   }

   private final static Set<Class<?>> acceptableExceptions = new HashSet<>();
   static
   {
      acceptableExceptions.add(BoundingBoxException.class);
      acceptableExceptions.add(IllegalArgumentException.class);
      acceptableExceptions.add(RuntimeException.class);
   }

   public static void registerFrameRandomGeneratorClasses(Class<?>... classes)
   {
      frameRandomGeneratorLibrary.addAll(Arrays.asList(classes));
   }

   public static void registerFramelessRandomGeneratorClasses(Class<?>... classes)
   {
      framelessRandomGeneratorLibrary.addAll(Arrays.asList(classes));
   }

   public static void registerFramelessTypesSmart(Class<?>... framelessMutableTypes)
   {
      for (Class<?> framelessMutableType : framelessMutableTypes)
         registerFramelessTypeSmart(framelessMutableType);
   }

   public static void registerFramelessTypeSmart(Class<?> framelessMutableType)
   {
      Class<?> framelessReadOnlyType = searchSuperInterfaceFromSimpleName(framelessMutableType.getSimpleName().replace(BASICS, READ_ONLY),
                                                                          framelessMutableType);
      GenericTypeBuilder framelessTypeBuilder = searchFramelessGenerator(framelessMutableType);

      Objects.requireNonNull(framelessReadOnlyType);
      registerFramelessType(framelessMutableType, framelessReadOnlyType, framelessTypeBuilder);
   }

   public static void registerFramelessType(Class<?> framelessMutableType, Class<?> framelessReadOnlyType, GenericTypeBuilder framelessTypeBuilder)
   {
      framelessTypesWithoutFrameEquivalent.addAll(Arrays.asList(framelessMutableType, framelessReadOnlyType));
      framelessTypeBuilders.put(framelessReadOnlyType, framelessTypeBuilder);
   }

   public static void registerFrameTypesSmart(Class<?>... mutableFrameMutableTypes)
   {
      for (Class<?> mutableFrameMutableType : mutableFrameMutableTypes)
         registerFrameTypeSmart(mutableFrameMutableType);
   }

   public static void registerFrameTypeSmart(Class<?> mutableFrameMutableType)
   {
      String mutableFrameMutableTypeName = mutableFrameMutableType.getSimpleName();

      String fixedFrameMutableTypeName = mutableFrameMutableTypeName.replace(FRAME, FIXED_FRAME);
      Class<?> fixedFrameMutableType = searchSuperInterfaceFromSimpleName(fixedFrameMutableTypeName, mutableFrameMutableType);

      String frameReadOnlyTypeName = mutableFrameMutableTypeName.replace(BASICS, READ_ONLY);
      Class<?> frameReadOnlyType = searchSuperInterfaceFromSimpleName(frameReadOnlyTypeName, fixedFrameMutableType);

      String framelessMutableTypeName = mutableFrameMutableTypeName.replace(FRAME, "");
      Class<?> framelessMutableType = searchSuperInterfaceFromSimpleName(framelessMutableTypeName, fixedFrameMutableType);

      String framelessReadOnlyTypeName = framelessMutableType.getSimpleName().replace(BASICS, READ_ONLY);
      Class<?> framelessReadOnlyType = searchSuperInterfaceFromSimpleName(framelessReadOnlyTypeName, framelessMutableType);

      RandomFrameTypeBuilder<?> frameTypeBuilder = searchFrameGenerator(mutableFrameMutableType);
      GenericTypeBuilder framelessTypeBuilder = searchFramelessGenerator(framelessMutableType);

      Objects.requireNonNull(fixedFrameMutableType);
      Objects.requireNonNull(frameReadOnlyType);
      Objects.requireNonNull(framelessMutableType);
      Objects.requireNonNull(framelessReadOnlyType);

      registerFrameType(mutableFrameMutableType,
                        fixedFrameMutableType,
                        frameReadOnlyType,
                        framelessMutableType,
                        framelessReadOnlyType,
                        frameTypeBuilder,
                        framelessTypeBuilder);
   }

   public static void registerReadOnlyFrameTypeSmart(Class<?>... frameReadOnlyTypes)
   {
      for (Class<?> frameReadOnlyType : frameReadOnlyTypes)
         registerReadOnlyFrameTypeSmart(frameReadOnlyType);
   }

   public static void registerReadOnlyFrameTypeSmart(Class<?> frameReadOnlyType)
   {
      Class<?> framelessReadOnlyType = searchSuperInterfaceFromSimpleName(frameReadOnlyType.getSimpleName().replace(FRAME, ""), frameReadOnlyType);
      Objects.requireNonNull(framelessReadOnlyType);

      RandomFrameTypeBuilder<?> frameTypeBuilder = searchFrameGenerator(frameReadOnlyType);
      GenericTypeBuilder framelessTypeBuilder = searchFramelessGenerator(framelessReadOnlyType);

      registerFrameType(null, null, frameReadOnlyType, null, framelessReadOnlyType, frameTypeBuilder, framelessTypeBuilder);
   }

   public static void registerFrameType(Class<?> mutableFrameMutableType, Class<?> fixedFrameMutableType, Class<?> frameReadOnlyType,
                                        Class<?> framelessMutableType, Class<?> framelessReadOnlyType, RandomFrameTypeBuilder<?> frameTypeBuilder,
                                        GenericTypeBuilder framelessTypeBuilder)
   {
      framelessTypesToFrameTypesTable.put(framelessReadOnlyType, frameReadOnlyType);
      if (fixedFrameMutableType != null && framelessMutableType != null)
         framelessTypesToFrameTypesTable.put(framelessMutableType, fixedFrameMutableType);

      frameTypeBuilders.put(frameReadOnlyType, frameTypeBuilder);
      framelessTypeBuilders.put(framelessReadOnlyType, framelessTypeBuilder);

      frameReadOnlyTypes.add(frameReadOnlyType);
      if (fixedFrameMutableType != null)
         fixedFrameMutableTypes.add(fixedFrameMutableType);
      if (mutableFrameMutableType != null)
         mutableFrameMutableTypes.add(mutableFrameMutableType);
   }

   private EuclidFrameAPITester()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    *
    * @param typeWithFrameMethods     refers to the type to be tested. This asserts that
    *                                 {@code typeWithFrameMethods} properly has all the methods
    *                                 necessary to properly overload {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods refers to the type declaring methods with frameless objects that
    *                                 are to be overloaded.
    * @param assertAllCombinations    when {@code false}, this asserts that for each method in
    *                                 {@code typeWithFramelessMethods} there is one overloading method
    *                                 in {@code typeWithFrameMethods} with all the arguments using the
    *                                 equivalent frame type. When {@code true}, this asserts that for
    *                                 each method in {@code typeWithFramelessArguments},
    *                                 {@code typeWithFrameMethods} overloads it with all the possible
    *                                 combinations of frame & frameless arguments, except for the
    *                                 original frameless signature.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations)
   {
      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, 1);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    *
    * @param typeWithFrameMethods          refers to the type to be tested. This asserts that
    *                                      {@code typeWithFrameMethods} properly has all the methods
    *                                      necessary to properly overload
    *                                      {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods      refers to the type declaring methods with frameless objects
    *                                      that are to be overloaded.
    * @param assertAllCombinations         when {@code false}, this asserts that for each method in
    *                                      {@code typeWithFramelessMethods} there is one overloading
    *                                      method in {@code typeWithFrameMethods} with all the
    *                                      arguments using the equivalent frame type. When
    *                                      {@code true}, this asserts that for each method in
    *                                      {@code typeWithFramelessArguments},
    *                                      {@code typeWithFrameMethods} overloads it with all the
    *                                      possible combinations of frame & frameless arguments, except
    *                                      for the original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *                                      {@code typeWithFramelessMethods}.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments)
   {
      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, minNumberOfFramelessArguments, m -> true);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    *
    * @param typeWithFrameMethods          refers to the type to be tested. This asserts that
    *                                      {@code typeWithFrameMethods} properly has all the methods
    *                                      necessary to properly overload
    *                                      {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods      refers to the type declaring methods with frameless objects
    *                                      that are to be overloaded.
    * @param assertAllCombinations         when {@code false}, this asserts that for each method in
    *                                      {@code typeWithFramelessMethods} there is one overloading
    *                                      method in {@code typeWithFrameMethods} with all the
    *                                      arguments using the equivalent frame type. When
    *                                      {@code true}, this asserts that for each method in
    *                                      {@code typeWithFramelessArguments},
    *                                      {@code typeWithFrameMethods} overloads it with all the
    *                                      possible combinations of frame & frameless arguments, except
    *                                      for the original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *                                      {@code typeWithFramelessMethods}.
    * @param framelessMethodsToIgnore      map containing the name and argument types of the methods in
    *                                      {@code typeWithFramelessMethods} to be ignored in this test.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments, Map<String, Class<?>[]> framelessMethodsToIgnore)
   {
      Predicate<Method> methodFilter = new Predicate<Method>()
      {
         @Override
         public boolean test(Method m)
         {
            for (Entry<String, Class<?>[]> methodToIgnore : framelessMethodsToIgnore.entrySet())
            {
               if (m.getName().equals(methodToIgnore.getKey()))
               {
                  if (Arrays.equals(m.getParameterTypes(), methodToIgnore.getValue()))
                     return false;
               }
            }
            return true;
         }
      };

      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, minNumberOfFramelessArguments, methodFilter);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2DBasics} and/or
    * {@code FixedFrameTuple2DBasics}.
    *
    * @param typeWithFrameMethods          refers to the type to be tested. This asserts that
    *                                      {@code typeWithFrameMethods} properly has all the methods
    *                                      necessary to properly overload
    *                                      {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods      refers to the type declaring methods with frameless objects
    *                                      that are to be overloaded.
    * @param assertAllCombinations         when {@code false}, this asserts that for each method in
    *                                      {@code typeWithFramelessMethods} there is one overloading
    *                                      method in {@code typeWithFrameMethods} with all the
    *                                      arguments using the equivalent frame type. When
    *                                      {@code true}, this asserts that for each method in
    *                                      {@code typeWithFramelessArguments},
    *                                      {@code typeWithFrameMethods} overloads it with all the
    *                                      possible combinations of frame & frameless arguments, except
    *                                      for the original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *                                      {@code typeWithFramelessMethods}.
    * @param framelessMethodFilter         custom filter used on the methods of
    *                                      {@code typeWithFramelessMethods}. The assertions are
    *                                      performed on the methods for which
    *                                      {@code framelessMethodFilter.test(method)} returns
    *                                      {@code true}.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments, Predicate<Method> framelessMethodFilter)
   {
      // The frame methods are all the methods from 'typeWithFramelessMethods' that have at least one geometry argument.
      List<Method> framelessMethods = keepOnlyMethodsWithAtLeastNFramelessArguments(typeWithFramelessMethods.getMethods(), minNumberOfFramelessArguments);

      for (Method framelessMethod : framelessMethods)
      {
         if (Modifier.isStatic(framelessMethod.getModifiers()))
            continue;

         if (!framelessMethodFilter.test(framelessMethod))
            continue;

         // Creating all the expected combinations
         List<Class<?>[]> expectedMethodSignatures = createExpectedMethodSignaturesWithFrameArgument(framelessMethod, assertAllCombinations);

         for (Class<?>[] expectedMethodSignature : expectedMethodSignatures)
         {
            assertMethodOverloadedWithSpecificSignature(typeWithFrameMethods,
                                                        typeWithFramelessMethods,
                                                        framelessMethod,
                                                        expectedMethodSignature,
                                                        typeWithFrameMethods);
         }
      }
   }

   /**
    * Asserts, using reflection, that the methods, that are public and static, in
    * {@code typeHoldingStaticMethodsToTest} are properly checking and/or setting reference frames of
    * their arguments.
    * <p>
    * This assertion expects methods to be declaring arguments as read-only to inform that they are
    * used as input only, and as mutable to inform that they are the output(s).
    * </p>
    * <p>
    * Note that this does not perform any assertion for methods with only 1 frame argument.
    * </p>
    * <p>
    * This expects methods to throw a {@link ReferenceFrameMismatchException} to indicate that the
    * operation cannot be performed because at least two arguments are expressed in a different
    * reference frame.
    * </p>
    *
    * @param typeDeclaringStaticMethodsToTest the type in which the methods are to be tested.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest) throws Throwable
   {
      assertStaticMethodsCheckReferenceFrame(typeDeclaringStaticMethodsToTest, m -> true);
   }

   /**
    * Asserts, using reflection, that the methods, that are public and static, in
    * {@code typeHoldingStaticMethodsToTest} are properly checking and/or setting reference frames of
    * their arguments.
    * <p>
    * This assertion expects methods to be declaring arguments as read-only to inform that they are
    * used as input only, and as mutable to inform that they are the output(s).
    * </p>
    * <p>
    * Note that this does not perform any assertion for methods with only 1 frame argument.
    * </p>
    * <p>
    * This expects methods to throw a {@link ReferenceFrameMismatchException} to indicate that the
    * operation cannot be performed because at least two arguments are expressed in a different
    * reference frame.
    * </p>
    *
    * @param typeDeclaringStaticMethodsToTest the type in which the methods are to be tested.
    * @param methodFilter                     custom filter used on the methods. The assertions are
    *                                         performed on the methods for which
    *                                         {@code methodFilter.test(method)} returns {@code true}.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, Predicate<Method> methodFilter) throws Throwable
   {
      // We need at least 2 frame arguments to assert anything.
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(typeDeclaringStaticMethodsToTest.getMethods(), 2);
      // We keep only the public & static methods
      frameMethods = frameMethods.stream().filter(m -> Modifier.isStatic(m.getModifiers())).filter(m -> Modifier.isPublic(m.getModifiers()))
                                 .collect(Collectors.toList());
      // Apply the custom filter
      frameMethods = frameMethods.stream().filter(methodFilter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < FRAME_CHECK_ITERATIONS; iteration++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

         // First check that the method is fine with all the arguments in the same frame.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            try
            {
               invokeStaticMethod(frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }
         }

         // Check that the method checks the reference frames.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();

            int numberOfArgumentsToTest = 0;
            for (Class<?> parameterType : parameterTypes)
            {
               if (!isFrameOfFrameTypeMutable(parameterType))
                  numberOfArgumentsToTest++;
            }
            int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToTest);

            for (int i = 1; i < numberOfCombinations - 1; i++)
            {
               Object[] parameters = new Object[parameterTypes.length];
               int currentByte = 0;

               for (int j = 0; j < parameterTypes.length; j++)
               {
                  Class<?> parameterType = parameterTypes[j];
                  boolean mutateFrame = !isFrameOfFrameTypeMutable(parameterType);

                  if (!mutateFrame)
                  {
                     parameters[j] = instantiateParameterType(frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = instantiateParameterType(frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeStaticMethod(frameMethod, parameters);
                  String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
                  message += "\nType being tested: " + typeDeclaringStaticMethodsToTest.getSimpleName();
                  message += "\nMethod: " + getMethodSimpleName(frameMethod);
                  message += "\nArguments used: " + Arrays.toString(parameters);
                  message += "\nArgument types: " + getArgumentTypeString(parameters);
                  throw new AssertionError(message);
               }
               catch (ReferenceFrameMismatchException e)
               {
                  // Good
               }
               catch (Throwable t)
               {
                  if (!isExceptionAcceptable(t))
                     throw t;
               }
            }
         }

         // Check that the frame of each mutable is changed (optional)
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isMutableFrameMutableType(parameterType))
                  parameters[i] = instantiateParameterType(frameB, parameterType);
               else
                  parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            try
            {
               invokeStaticMethod(frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
               else
                  continue;
            }

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isMutableFrameMutableType(parameterType))
               {
                  ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                  if (newFrame != frameA)
                  {
                     String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + (i + 1) + "th parameter.";
                     message += "\nType being tested: " + typeDeclaringStaticMethodsToTest.getSimpleName();
                     message += "\nArguments used: " + Arrays.toString(parameters);
                     message += "\nArgument types: " + getArgumentTypeString(parameters);
                     throw new AssertionError(message);
                  }
               }
            }
         }

         // Check for methods returning a frame type that the reference frame is properly set.
         for (Method frameMethod : methodsWithReturnFrameType)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            Object result = null;

            try
            {
               result = invokeStaticMethod(frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }

            if (result == null)
               continue;

            ReferenceFrame resultFrame = ((ReferenceFrameHolder) result).getReferenceFrame();
            if (resultFrame != frameA)
            {
               String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not set the frame of the result.";
               message += "\nType being tested: " + typeDeclaringStaticMethodsToTest.getSimpleName();
               message += "\nArguments used: " + Arrays.toString(parameters);
               message += "\nArgument types: " + getArgumentTypeString(parameters);
               message += "\nResult: " + result;
               throw new AssertionError(message);
            }
         }
      }
   }

   /**
    * Asserts, using reflection, that the methods, that are public and non-static, in the created
    * instance from {@code frameTypeBuilder} are properly checking and/or setting reference frames of
    * their arguments.
    * <p>
    * This assertion expects methods to be declaring arguments as read-only to inform that they are
    * used as input only, and as mutable to inform that they are the output(s).
    * </p>
    * <p>
    * This expects methods to throw a {@link ReferenceFrameMismatchException} to indicate that the
    * operation cannot be performed because at least one argument with an immutable frame is expressed
    * in a different reference frame.
    * </p>
    *
    * @param frameTypeBuilder builder used to generate an instance of the type to be tested.
    * @param methodFilter     custom filter used on the methods. The assertions are performed on the
    *                         methods for which {@code methodFilter.test(method)} returns {@code true}.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertMethodsOfReferenceFrameHolderCheckReferenceFrame(RandomFrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder,
                                                                             Predicate<Method> methodFilter)
         throws Throwable
   {
      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(random, worldFrame).getClass();

      // We need at least 1 frame arguments to assert anything.
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(frameType.getMethods(), 1);
      // We keep only the public & static methods
      frameMethods = frameMethods.stream().filter(m -> Modifier.isPublic(m.getModifiers())).collect(Collectors.toList());
      // Apply the custom filter
      frameMethods = frameMethods.stream().filter(methodFilter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < FRAME_CHECK_ITERATIONS; iteration++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

         // First check that the method is fine with the holder and all the arguments in the same frame.
         for (Method frameMethod : frameMethods)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            try
            {
               invokeMethod(frameObject, frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }
         }

         // Check that the method checks the reference frames.
         for (Method frameMethod : frameMethods)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();

            int numberOfArgumentsToTest = 0;
            for (Class<?> parameterType : parameterTypes)
            {
               if (!isFrameOfFrameTypeMutable(parameterType))
                  numberOfArgumentsToTest++;
            }
            int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToTest);

            for (int i = 1; i < numberOfCombinations; i++)
            {
               Object[] parameters = new Object[parameterTypes.length];
               int currentByte = 0;

               for (int j = 0; j < parameterTypes.length; j++)
               {
                  Class<?> parameterType = parameterTypes[j];
                  boolean mutateFrame = !isFrameOfFrameTypeMutable(parameterType);

                  if (!mutateFrame)
                  {
                     parameters[j] = instantiateParameterType(frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = instantiateParameterType(frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeMethod(frameObject, frameMethod, parameters);
                  String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
                  message += "\nType being tested: " + frameType.getSimpleName();
                  message += "\nMethod: " + getMethodSimpleName(frameMethod);
                  message += "\nArguments used: " + Arrays.toString(parameters);
                  message += "\nArgument types: " + getArgumentTypeString(parameters);
                  throw new AssertionError(message);
               }
               catch (ReferenceFrameMismatchException e)
               {
                  // Good
               }
               catch (Throwable t)
               {
                  if (!(t instanceof ReferenceFrameMismatchException))
                     throw t;
               }
            }
         }

         // Check that the frame of each mutable is changed (optional)
         for (Method frameMethod : frameMethods)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isMutableFrameMutableType(parameterType))
                  parameters[i] = instantiateParameterType(frameB, parameterType);
               else
                  parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            try
            {
               invokeMethod(frameObject, frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
               else
                  continue;
            }

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isMutableFrameMutableType(parameterType))
               {
                  ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                  if (newFrame != frameA)
                  {
                     String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + (i + 1) + "th parameter.";
                     message += "\nType being tested: " + frameType.getSimpleName();
                     message += "\nArguments used: " + Arrays.toString(parameters);
                     message += "\nArgument types: " + getArgumentTypeString(parameters);
                     throw new AssertionError(message);
                  }
               }
            }
         }

         // Check for methods returning a frame type that the reference frame is properly set.
         for (Method frameMethod : methodsWithReturnFrameType)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            Object result = null;

            try
            {
               result = invokeMethod(frameObject, frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }

            if (result == null)
               continue;

            ReferenceFrame resultFrame = ((ReferenceFrameHolder) result).getReferenceFrame();
            if (resultFrame != frameA)
            {
               String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not set the frame of the result.";
               message += "\nType being tested: " + frameType.getSimpleName();
               message += "\nArguments used: " + Arrays.toString(parameters);
               message += "\nArgument types: " + getArgumentTypeString(parameters);
               message += "\nResult: " + result;
               throw new AssertionError(message);
            }
         }
      }
   }

   /**
    * Assuming the type {@code typeWithFrameMethodsToTest} declares the same static methods as declared
    * in {@code typeWithFramlessMethods} with the difference of dealing with reference frame holders,
    * this method asserts that the methods in {@code typeWithFrameMethodsToTest} does not change the
    * underlying algorithms.
    * <p>
    * For each method declared in {@code typeWithFrameMethodsToTest}, this methods searched for the
    * equivalent method in {@code typeWithFramelessMethods} and the methods from both classes are
    * invoked to compare the output.
    * </p>
    *
    * @param typeWithFrameMethodsToTest the type in which the methods are to be tested.
    * @param typeWithFramelessMethods   the type declaring the methods against which the methods from
    *                                   {@code typeWithFrameMethodsToTest} are to be compared.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods)
   {
      assertStaticMethodsPreserveFunctionality(typeWithFrameMethodsToTest, typeWithFramelessMethods, m -> true);
   }

   /**
    * Assuming the type {@code typeWithFrameMethodsToTest} declares the same static methods as declared
    * in {@code typeWithFramlessMethods} with the difference of dealing with reference frame holders,
    * this method asserts that the methods in {@code typeWithFrameMethodsToTest} does not change the
    * underlying algorithms.
    * <p>
    * For each method declared in {@code typeWithFrameMethodsToTest}, this methods searched for the
    * equivalent method in {@code typeWithFramelessMethods} and the methods from both classes are
    * invoked to compare the output.
    * </p>
    *
    * @param typeWithFrameMethodsToTest the type in which the methods are to be tested.
    * @param typeWithFramelessMethods   the type declaring the methods against which the methods from
    *                                   {@code typeWithFrameMethodsToTest} are to be compared.
    * @param methodFilter               custom filter used on the methods. The assertions are performed
    *                                   on the methods for which {@code methodFilter.test(method)}
    *                                   returns {@code true}.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods,
                                                               Predicate<Method> methodFilter)
   {
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(typeWithFrameMethodsToTest.getMethods(), 0);

      for (Method frameMethod : frameMethods)
      {
         if (!methodFilter.test(frameMethod))
            continue;

         String frameMethodName = frameMethod.getName();
         Class<?>[] frameMethodParameterTypes = frameMethod.getParameterTypes();
         Class<?>[] framelessMethodParameterTypes = new Class[frameMethodParameterTypes.length];

         for (int i = 0; i < framelessMethodParameterTypes.length; i++)
         {
            if (isFrameType(frameMethodParameterTypes[i]))
               framelessMethodParameterTypes[i] = findCorrespondingFramelessType(frameMethodParameterTypes[i]);
            else
               framelessMethodParameterTypes[i] = frameMethodParameterTypes[i];
         }

         for (int iteration = 0; iteration < FUNCTIONALITY_ITERATIONS; iteration++)
         {
            try
            {
               Method framelessMethod = typeWithFramelessMethods.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = instantiateParameterTypes(worldFrame, frameMethodParameterTypes);

               if (frameMethodParameters == null)
               {
                  if (DEBUG)
                  {
                     String message = "Could not instantiate the parameters for the method: " + getMethodSimpleName(frameMethod)
                           + ". The method is not tested.";
                     System.err.println(message);
                  }
                  break;
               }

               Object[] framelessMethodParameters = clone(frameMethodParameters);
               Throwable expectedException = null;
               Object framelessMethodReturnObject = null;
               Object frameMethodReturnObject = null;

               try
               {
                  framelessMethodReturnObject = invokeStaticMethod(framelessMethod, framelessMethodParameters);
               }
               catch (Throwable e)
               {
                  expectedException = e;
               }

               try
               {
                  frameMethodReturnObject = invokeStaticMethod(frameMethod, frameMethodParameters);
               }
               catch (Throwable e)
               {
                  if (e.getClass() != expectedException.getClass())
                  {
                     String message = "";
                     message += "The method: " + getMethodSimpleName(frameMethod);
                     message += "\ndid not throw the same exception as the original method: " + getMethodSimpleName(framelessMethod);
                     message += "\nExpected exception class: " + expectedException.getClass().getSimpleName();
                     message += "\nActual exception class: " + e.getClass().getSimpleName();
                     throw new AssertionError(message);
                  }
                  else
                  {
                     continue;
                  }
               }

               for (int i = 0; i < frameMethodParameters.length; i++)
               {
                  Object framelessParameter = framelessMethodParameters[i];
                  Object frameParameter = frameMethodParameters[i];

                  if (!epsilonEquals(framelessParameter, frameParameter, epsilon))
                  {
                     String message = "";
                     message += "Detected a frame method inconsistent with its original frameless method.";
                     message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                     message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                     message += "\nFrame arguments after call:\n" + Arrays.toString(frameMethodParameters);
                     message += "\nFrameless arguments after call:\n" + toStringAsFramelessObjects(framelessMethodParameters);
                     throw new AssertionError(message);
                  }
               }

               if (!epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
               {
                  String message = "";
                  message += "Detected a frame method inconsistent with its original frameless method.";
                  message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                  message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                  message += "\nFrame method returned:" + frameMethodReturnObject;
                  message += "\nFrameless method returned:" + toStringAsFramelessObject(framelessMethodReturnObject);
                  throw new AssertionError(message);
               }
            }
            catch (NoSuchMethodException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nCould not find the corresponding method: " + getMethodSimpleName(frameMethod);
                  message += "\nMethod is from type: " + typeWithFrameMethodsToTest.getSimpleName();
                  message += "\nSearched in: " + typeWithFramelessMethods.getSimpleName();
                  message += "\nSearched with argument type: " + getSimpleNames(framelessMethodParameterTypes);
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
            catch (SecurityException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nUnable to access method with name: " + frameMethodName + " and argument types: "
                        + getSimpleNames(framelessMethodParameterTypes);
                  message += "\nin type: " + typeWithFramelessMethods.getSimpleName();
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
         }
      }
   }

   /**
    * Assuming the type built by the {@code frameTypeBuilder} declares the same methods as declared in
    * the type built by {@code framelessTypeBuilder} with the difference of handling the reference
    * frame information, this method asserts that the methods the type built by the
    * {@code frameTypeBuilder} does not change the underlying algorithms.
    * <p>
    * For each method declared in the type built by the {@code frameTypeBuilder}, this methods searched
    * for the equivalent method in type built by the {@code framelessTypeBuilder} and the methods from
    * both classes are invoked to compare the output.
    * </p>
    *
    * @param frameTypeBuilder     the builder for creating instances of the frame object to test.
    * @param framelessTypeBuilber the builder for creating instances of the corresponding frameless
    *                             objects.
    * @param methodFilter         custom filter used on the methods. The assertions are performed on
    *                             the methods for which {@code methodFilter.test(method)} returns
    *                             {@code true}.
    */
   public static void assertFrameMethodsOfFrameHolderPreserveFunctionality(FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder,
                                                                           GenericTypeBuilder framelessTypeBuilber, Predicate<Method> methodFilter)
   {

      Class<? extends ReferenceFrameHolder> frameTypeToTest = frameTypeBuilder.newInstance(worldFrame, framelessTypeBuilber.newInstance(random)).getClass();
      Class<? extends Object> framelessType = framelessTypeBuilber.newInstance(random).getClass();

      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(frameTypeToTest.getMethods(), 0);

      for (Method frameMethod : frameMethods)
      {
         if (!methodFilter.test(frameMethod))
            continue;

         String frameMethodName = frameMethod.getName();
         Class<?>[] frameMethodParameterTypes = frameMethod.getParameterTypes();
         Class<?>[] framelessMethodParameterTypes = new Class[frameMethodParameterTypes.length];

         for (int i = 0; i < framelessMethodParameterTypes.length; i++)
         {
            if (isFrameType(frameMethodParameterTypes[i]))
               framelessMethodParameterTypes[i] = findCorrespondingFramelessType(frameMethodParameterTypes[i]);
            else
               framelessMethodParameterTypes[i] = frameMethodParameterTypes[i];
         }

         for (int iteration = 0; iteration < FUNCTIONALITY_ITERATIONS; iteration++)
         {
            Object framelessObject = framelessTypeBuilber.newInstance(random);
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(worldFrame, framelessObject);

            try
            {
               Method framelessMethod = framelessType.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = instantiateParameterTypes(worldFrame, frameMethodParameterTypes);

               if (frameMethodParameters == null)
               {
                  if (DEBUG)
                  {
                     String message = "Could not instantiate the parameters for the method: " + getMethodSimpleName(frameMethod)
                           + ". The method is not tested.";
                     System.err.println(message);
                  }
                  break;
               }

               Object[] framelessMethodParameters = clone(frameMethodParameters);
               Throwable expectedException = null;
               Object framelessMethodReturnObject = null;
               Object frameMethodReturnObject = null;

               try
               {
                  framelessMethodReturnObject = invokeMethod(framelessObject, framelessMethod, framelessMethodParameters);
               }
               catch (Throwable e)
               {
                  expectedException = e;
               }

               try
               {
                  frameMethodReturnObject = invokeMethod(frameObject, frameMethod, frameMethodParameters);
               }
               catch (Throwable e)
               {
                  if (expectedException == null || e.getClass() != expectedException.getClass())
                  {
                     String message = "";
                     message += "The method: " + getMethodSimpleName(frameMethod);
                     message += "\ndid not throw the same exception as the original method: " + getMethodSimpleName(framelessMethod);
                     message += "\nExpected exception class: " + (expectedException == null ? "none" : expectedException.getClass().getSimpleName());
                     message += "\nActual exception class: " + e.getClass().getSimpleName();
                     throw new AssertionError(message);
                  }
                  else
                  {
                     continue;
                  }
               }

               for (int i = 0; i < frameMethodParameters.length; i++)
               {
                  Object framelessParameter = framelessMethodParameters[i];
                  Object frameParameter = frameMethodParameters[i];

                  if (!epsilonEquals(framelessParameter, frameParameter, epsilon))
                  {
                     String message = "";
                     message += "Detected a frame method inconsistent with its original frameless method.";
                     message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                     message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                     message += "\nFrame arguments after call:\n" + Arrays.toString(frameMethodParameters);
                     message += "\nFrameless arguments after call:\n" + toStringAsFramelessObjects(framelessMethodParameters);
                     throw new AssertionError(message);
                  }
               }

               if (!epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
               {
                  String message = "";
                  message += "Detected a frame method inconsistent with its original frameless method.";
                  message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                  message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                  message += "\nFrame method returned:" + frameMethodReturnObject;
                  message += "\nFrameless method returned:" + toStringAsFramelessObject(framelessMethodReturnObject);
                  throw new AssertionError(message);
               }

               if (!epsilonEquals(framelessObject, frameObject, epsilon))
               {
                  String message = "";
                  message += "Detected a frame method inconsistent with its original frameless method.";
                  message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                  message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                  message += "\nFrame object after method call:" + frameObject;
                  message += "\nFrameless object after method call:" + framelessObject;
                  throw new AssertionError(message);
               }

            }
            catch (NoSuchMethodException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nCould not find the corresponding method: " + getMethodSimpleName(frameMethod);
                  message += "\nMethod is from type: " + frameTypeToTest.getSimpleName();
                  message += "\nSearched in: " + framelessType.getSimpleName();
                  message += "\nSearched with argument type: " + getSimpleNames(framelessMethodParameterTypes);
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
            catch (SecurityException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nUnable to access method with name: " + frameMethodName + " and argument types: "
                        + getSimpleNames(framelessMethodParameterTypes);
                  message += "\nin type: " + framelessType.getSimpleName();
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
            catch (RuntimeException e)
            {
               System.err.println("Problem when evaluating the method: "
                     + getMethodSimpleName(frameMethod.getReturnType(), frameMethodName, frameMethodParameterTypes));
               throw e;
            }
         }
      }
   }

   private static String toStringAsFramelessObjects(Object[] frameObjects)
   {
      String ret = "[";
      for (int i = 0; i < frameObjects.length; i++)
      {
         ret += toStringAsFramelessObject(frameObjects[i]);
         if (i < frameObjects.length - 1)
            ret += ", ";
         else
            ret += "]";
      }
      return ret;
   }

   private static String toStringAsFramelessObject(Object frameObject)
   {
      if (isFrameObject(frameObject))
      {
         return findCorrespondingFramelessType(frameObject.getClass()).cast(frameObject).toString();
      }
      else
      {
         return frameObject.toString();
      }
   }

   @SuppressWarnings("unchecked")
   private static <T> boolean epsilonEquals(Object framelessParameter, Object frameParameter, double epsilon)
   {
      if (framelessParameter == null && frameParameter == null)
         return true;

      if (framelessParameter != null ^ frameParameter != null)
         return false;

      if (framelessParameter instanceof Clearable && frameParameter instanceof Clearable)
      {
         if (((Clearable) framelessParameter).containsNaN() && ((Clearable) frameParameter).containsNaN())
            return true;
      }

      if (isVertexSupplier(frameParameter.getClass()))
      {
         try
         {
            Method epsilonEqualsMethod = frameParameter.getClass().getMethod("epsilonEquals", framelessParameter.getClass().getInterfaces()[0], double.class);
            boolean epsilonEqualsResult = (boolean) epsilonEqualsMethod.invoke(frameParameter, framelessParameter, epsilon);
            return epsilonEqualsResult;
         }
         catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            System.err.println("Something went wrong when invoking the epsilonEquals method for " + frameParameter.getClass().getSimpleName());
            System.err.println("Objects used as parameters: " + getArgumentTypeString(framelessParameter, epsilon));
            e.printStackTrace();
            throw new AssertionError(e);
         }

      }
      else if (isFramelessObject(framelessParameter))
      {
         if (!isFrameObject(frameParameter) && !isFramelessObject(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         if (isFrameObject(frameParameter))
         {
            Method epsilonEqualsMethod;
            try
            {
               epsilonEqualsMethod = Arrays.stream(frameParameter.getClass().getMethods()).filter(m -> m.getName().equals("epsilonEquals"))
                                           .filter(m -> m.getParameterTypes()[0] != Object.class) // Filters the method from FrameGeometryObject.
                                           .filter(m -> m.getParameterTypes()[0].isAssignableFrom(framelessParameter.getClass())).findFirst().orElse(null);
            }
            catch (SecurityException e)
            {
               throw new AssertionError(e);
            }

            boolean epsilonEqualsResult = false;

            try
            {
               epsilonEqualsResult = (boolean) epsilonEqualsMethod.invoke(frameParameter, framelessParameter, epsilon);
            }
            catch (IllegalAccessException | IllegalArgumentException e)
            {
               System.err.println("Something went wrong when invoking the epsilonEquals method for " + frameParameter.getClass().getSimpleName());
               System.err.println("Objects used as parameters: " + getArgumentTypeString(framelessParameter, epsilon));
               e.printStackTrace();
               throw new AssertionError(e);
            }
            catch (InvocationTargetException e)
            {
               throw new AssertionError(e.getCause());
            }
            return epsilonEqualsResult;
         }
         else
         {
            return ((EpsilonComparable<T>) framelessParameter).epsilonEquals((T) frameParameter, epsilon);
         }
      }

      if (isFrameObject(framelessParameter))
      {
         if (!isFrameObject(frameParameter) && !isFramelessObject(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         if (isFrameObject(frameParameter))
            return ((EpsilonComparable<T>) framelessParameter).epsilonEquals((T) frameParameter, epsilon);
      }

      if (Double.TYPE.isInstance(framelessParameter) || Float.TYPE.isInstance(framelessParameter))
      {
         if (!Double.TYPE.isInstance(frameParameter) && !Float.TYPE.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return EuclidCoreTools.epsilonEquals((double) framelessParameter, (double) frameParameter, epsilon);
      }

      if (Integer.TYPE.isInstance(framelessParameter) || Long.TYPE.isInstance(framelessParameter))
      {
         if (!Integer.TYPE.isInstance(frameParameter) && !Long.TYPE.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return (long) framelessParameter == (long) frameParameter;
      }

      if (Double.class.isInstance(framelessParameter) || Float.class.isInstance(framelessParameter))
      {
         if (!Double.class.isInstance(frameParameter) && !Float.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         double framelessDouble = ((Number) framelessParameter).doubleValue();
         double frameDouble = ((Number) frameParameter).doubleValue();
         return Double.compare(framelessDouble, frameDouble) == 0 || EuclidCoreTools.epsilonEquals(framelessDouble, frameDouble, epsilon);
      }

      if (Integer.class.isInstance(framelessParameter) || Long.class.isInstance(framelessParameter))
      {
         if (!Integer.class.isInstance(frameParameter) && !Long.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return ((Number) framelessParameter).longValue() == ((Number) frameParameter).longValue();
      }

      if (Boolean.class.isInstance(framelessParameter))
      {
         if (!Boolean.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return (boolean) framelessParameter == (boolean) frameParameter;
      }

      if (framelessParameter instanceof EpsilonComparable && frameParameter instanceof EpsilonComparable)
      {
         return ((EpsilonComparable<T>) framelessParameter).epsilonEquals((T) frameParameter, epsilon);
      }

      if (framelessParameter instanceof List)
      {
         if (frameParameter instanceof List)
         {
            List<?> framelessList = (List<?>) framelessParameter;
            List<?> frameList = (List<?>) frameParameter;

            if (framelessList.size() != frameList.size())
               return false;

            for (int i = 0; i < framelessList.size(); i++)
            {
               if (!epsilonEquals(framelessList.get(i), frameList.get(i), epsilon))
                  return false;
            }
            return true;
         }
         else
         {
            throw new RuntimeException("Reached unexpected state.");
         }
      }

      if (framelessParameter instanceof DenseMatrix64F && frameParameter instanceof DenseMatrix64F)
      {
         return MatrixFeatures.isEquals((DenseMatrix64F) framelessParameter, (DenseMatrix64F) frameParameter, epsilon);
      }

      if (framelessParameter instanceof int[] && frameParameter instanceof int[])
      {
         int[] framelessArray = (int[]) framelessParameter;
         int[] frameArray = (int[]) frameParameter;

         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (Float.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }

      if (framelessParameter instanceof float[] && frameParameter instanceof float[])
      {
         float[] framelessArray = (float[]) framelessParameter;
         float[] frameArray = (float[]) frameParameter;

         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (Float.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }

      if (framelessParameter instanceof double[] && frameParameter instanceof double[])
      {
         double[] framelessArray = (double[]) framelessParameter;
         double[] frameArray = (double[]) frameParameter;

         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (Double.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }

      if (framelessParameter instanceof String && frameParameter instanceof String)
         return true;

      if (framelessParameter instanceof Class && frameParameter instanceof Class)
         return true;

      if (framelessParameter.getClass().isArray() && frameParameter.getClass().isArray())
      {
         Object[] framelessArray = (Object[]) framelessParameter;
         Object[] frameArray = (Object[]) frameParameter;
         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (!epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }

      throw new RuntimeException("Did not expect the following types: " + framelessParameter.getClass().getSimpleName() + " & "
            + frameParameter.getClass().getSimpleName());
   }

   private static boolean isMutableFrameMutableType(Class<?> frameType)
   {
      return mutableFrameMutableTypes.contains(frameType) && !fixedFrameMutableTypes.contains(frameType);
   }

   private static boolean isFrameOfFrameTypeMutable(Class<?> frameType)
   {
      return !fixedFrameMutableTypes.contains(frameType) && !frameReadOnlyTypes.contains(frameType);
   }

   private static Object invokeStaticMethod(Method frameMethod, Object[] parameters) throws Throwable
   {
      try
      {
         return frameMethod.invoke(null, parameters);
      }
      catch (IllegalAccessException | IllegalArgumentException e)
      {
         System.err.println("Something went wrong when invoking the static method: " + getMethodSimpleName(frameMethod));
         System.err.println("Objects used as parameters: " + getArgumentTypeString(parameters));
         e.printStackTrace();
         throw e;
      }
      catch (InvocationTargetException e)
      {
         throw e.getCause();
      }
   }

   private static Object invokeMethod(Object methodHolder, Method frameMethod, Object[] parameters) throws Throwable
   {
      try
      {
         return frameMethod.invoke(methodHolder, parameters);
      }
      catch (IllegalAccessException | IllegalArgumentException e)
      {
         System.err.println("Something went wrong when invoking the method: " + getMethodSimpleName(frameMethod));
         System.err.println("Objects used as parameters: " + getArgumentTypeString(parameters));
         e.printStackTrace();
         throw e;
      }
      catch (InvocationTargetException e)
      {
         throw e.getCause();
      }
   }

   private static boolean isExceptionAcceptable(Throwable t)
   {
      return acceptableExceptions.stream().filter(c -> c.isAssignableFrom(t.getClass())).findAny().isPresent();
   }

   private static String getArgumentTypeString(Object... arguments)
   {
      String string = "";
      for (int i = 0; i < arguments.length; i++)
      {
         string += arguments[i].getClass().getSimpleName();
         if (i < arguments.length - 1)
            string += ", ";
      }
      return string;
   }

   private static void assertMethodOverloadedWithSpecificSignature(Class<?> typeWithOverloadingMethods, Class<?> typeWithOriginalMethod, Method originalMethod,
                                                                   Class<?>[] overloadingSignature, Class<?> typeToSearchIn)
         throws SecurityException
   {
      try
      {
         Method overloadingMethod = typeToSearchIn.getMethod(originalMethod.getName(), overloadingSignature);
         Class<?> originalReturnType = originalMethod.getReturnType();
         Class<?> overloadingReturnType = overloadingMethod.getReturnType();

         { // Assert the return type is proper
            if (originalReturnType == null && overloadingReturnType != null)
            {
               String message = "Inconsistency found in the return type.";
               message += "\nOriginal method: " + getMethodSimpleName(originalMethod);
               message += "\nOverloading method: " + getMethodSimpleName(overloadingMethod);
               message += "\nOriginal type declaring method: " + typeWithOriginalMethod.getSimpleName();
               message += "\nType overloading original: " + typeWithOverloadingMethods.getSimpleName();
               throw new AssertionError(message);
            }

            if (overloadingReturnType.equals(originalReturnType))
               return;

            if (overloadingReturnType.isAssignableFrom(findCorrespondingFrameType(originalReturnType)))
               throw new AssertionError("Unexpected return type: expected: " + findCorrespondingFrameType(originalReturnType).getSimpleName() + ", actual: "
                     + overloadingReturnType.getSimpleName());
         }
      }
      catch (NoSuchMethodException e)
      {
         throw new AssertionError("The original method in " + typeWithOriginalMethod.getSimpleName() + ":\n" + getMethodSimpleName(originalMethod)
               + "\nis not properly overloaded, expected to find in " + typeToSearchIn.getSimpleName() + ":\n"
               + getMethodSimpleName(originalMethod.getReturnType(), originalMethod.getName(), overloadingSignature));
      }
   }

   private static String getMethodSimpleName(Method method)
   {
      return getMethodSimpleName(method.getReturnType(), method.getName(), method.getParameterTypes());
   }

   private static String getMethodSimpleName(Class<?> returnType, String methodName, Class<?>[] parameterTypes)
   {
      String returnTypeName = returnType == null ? "void" : returnType.getSimpleName();
      return returnTypeName + " " + methodName + "(" + getSimpleNames(parameterTypes) + ")";
   }

   private static String getSimpleNames(Class<?>[] types)
   {
      String ret = Arrays.stream(types).map(t -> t.getSimpleName()).collect(Collectors.toList()).toString();
      return ret.substring(1, ret.length() - 1);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFrameArguments(Method[] methodsToFilter, int minNumberOfFrameArguments)
   {
      return keepOnlyMethodsWithAtLeastNFrameArguments(Arrays.asList(methodsToFilter), minNumberOfFrameArguments);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFrameArguments(List<Method> methodsToFilter, int minNumberOfFrameArguments)
   {
      return methodsToFilter.stream().filter(m -> methodHasAtLeastNFrameArguments(m, minNumberOfFrameArguments)).collect(Collectors.toList());
   }

   private static boolean methodHasAtLeastNFrameArguments(Method method, int minNumberOfFrameArguments)
   {
      int numberOfFrameArguments = 0;

      for (Class<?> parameterType : method.getParameterTypes())
      {
         if (isFrameType(parameterType))
         {
            numberOfFrameArguments++;
            if (numberOfFrameArguments >= minNumberOfFrameArguments)
               return true;
         }
      }

      return numberOfFrameArguments >= minNumberOfFrameArguments;
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFramelessArguments(Method[] methodsToFilter, int minNumberOfFramelessArguments)
   {
      return keepOnlyMethodsWithAtLeastNFramelessArguments(Arrays.asList(methodsToFilter), minNumberOfFramelessArguments);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFramelessArguments(List<Method> methodsToFilter, int minNumberOfFramelessArguments)
   {
      return methodsToFilter.stream().filter(m -> methodHasAtLeastNFramelessArguments(m, minNumberOfFramelessArguments)).collect(Collectors.toList());
   }

   private static boolean methodHasAtLeastNFramelessArguments(Method method, int minNumberOfFramelessArguments)
   {
      int numberOfFramelessArguments = 0;

      for (Class<?> parameterType : method.getParameterTypes())
      {
         if (isFramelessTypeWithFrameEquivalent(parameterType))
         {
            numberOfFramelessArguments++;
            if (numberOfFramelessArguments >= minNumberOfFramelessArguments)
               return true;
         }
      }

      return numberOfFramelessArguments >= minNumberOfFramelessArguments;
   }

   private static List<Class<?>[]> createExpectedMethodSignaturesWithFrameArgument(Method methodWithoutFrameArguments, boolean createAllCombinations)
   {
      Class<?>[] framelessMethodArgumentTypes = methodWithoutFrameArguments.getParameterTypes();
      List<Class<?>[]> expectedMethodSignatures = new ArrayList<>();

      if (!createAllCombinations)
      {
         Class<?>[] combination = new Class<?>[framelessMethodArgumentTypes.length];
         System.arraycopy(framelessMethodArgumentTypes, 0, combination, 0, combination.length);

         for (int k = 0; k < combination.length; k++)
         {
            if (isFramelessTypeWithFrameEquivalent(combination[k]))
               combination[k] = findCorrespondingFrameType(combination[k]);
         }
         expectedMethodSignatures.add(combination);
      }
      else
      {
         int numberOfArgumentsToOverload = (int) Arrays.stream(framelessMethodArgumentTypes).filter(t -> isFramelessTypeWithFrameEquivalent(t)).count();
         int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToOverload);

         for (int i = 0; i < numberOfCombinations; i++)
         {
            Class<?>[] combination = new Class<?>[framelessMethodArgumentTypes.length];
            System.arraycopy(framelessMethodArgumentTypes, 0, combination, 0, combination.length);
            int currentByte = 0;

            for (int k = 0; k < combination.length; k++)
            {
               if (isFramelessTypeWithFrameEquivalent(combination[k]))
               {
                  int mask = (int) Math.pow(2, currentByte);
                  if ((i & mask) != 0)
                     combination[k] = findCorrespondingFrameType(combination[k]);
                  currentByte++;
               }
            }
            expectedMethodSignatures.add(combination);
         }

         // Remove the original method from the combinations
         for (int combinationIndex = 0; combinationIndex < expectedMethodSignatures.size(); combinationIndex++)
         {
            if (Arrays.equals(framelessMethodArgumentTypes, expectedMethodSignatures.get(combinationIndex)))
            {
               expectedMethodSignatures.remove(combinationIndex);
               break;
            }
         }
      }
      return expectedMethodSignatures;
   }

   private static Class<?> findCorrespondingFrameType(Class<?> framelessType)
   {
      if (framelessType.isArray())
         return Array.newInstance(findCorrespondingFrameType(framelessType.getComponentType()), 0).getClass();

      if (!isFramelessTypeWithFrameEquivalent(framelessType))
         throw new IllegalArgumentException("Cannot handle the following type: " + framelessType.getSimpleName());

      Class<?> frameType = null;

      for (Entry<Class<?>, Class<?>> entry : framelessTypesToFrameTypesTable.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(framelessType))
            continue;

         if (frameType == null || frameType.isAssignableFrom(entry.getValue()))
            frameType = entry.getValue();
      }

      if (frameType == null)
         throw new RuntimeException("Could not find the corresponding frame type for: " + framelessType.getSimpleName());

      return frameType;
   }

   private static Class<?> findCorrespondingFramelessType(Class<?> frameType)
   {
      if (frameType.isArray())
         return Array.newInstance(findCorrespondingFramelessType(frameType.getComponentType()), 0).getClass();

      if (!isFrameType(frameType))
         throw new IllegalArgumentException("Cannot handle the following type: " + frameType.getSimpleName());

      Class<?> framelessType = null;

      for (Entry<Class<?>, Class<?>> entry : framelessTypesToFrameTypesTable.entrySet())
      {
         if (!entry.getValue().isAssignableFrom(frameType))
            continue;

         if (framelessType == null || framelessType.isAssignableFrom(entry.getKey()))
            framelessType = entry.getKey();
      }

      if (framelessType == null)
         throw new RuntimeException("Could not find the corresponding frameless type for: " + frameType.getSimpleName());

      return framelessType;
   }

   private static boolean isFrameObject(Object object)
   {
      return isFrameType(object.getClass());
   }

   private static boolean isFrameType(Class<?> type)
   {
      for (Class<?> frameType : framelessTypesToFrameTypesTable.values())
      {
         if (frameType.isAssignableFrom(type))
            return true;
      }
      return false;
   }

   private static boolean isFramelessObject(Object object)
   {
      return isFramelessType(object.getClass());
   }

   private static boolean isFramelessType(Class<?> type)
   {
      if (ReferenceFrameHolder.class.isAssignableFrom(type))
         return false;

      for (Class<?> framelessType : framelessTypesToFrameTypesTable.keySet())
      {
         if (framelessType.isAssignableFrom(type))
            return true;
      }
      return false;
   }

   private static boolean isFramelessTypeWithFrameEquivalent(Class<?> type)
   {
      if (type.isArray())
         return isFramelessTypeWithFrameEquivalent(type.getComponentType());
      return isFramelessType(type) && !framelessTypesWithoutFrameEquivalent.contains(type);
   }

   private static Object[] clone(Object[] parametersToClone)
   {
      Object[] clone = (Object[]) Array.newInstance(parametersToClone.getClass().getComponentType(), parametersToClone.length);

      for (int i = 0; i < parametersToClone.length; i++)
      {
         Class<? extends Object> parameterType = parametersToClone[i].getClass();

         if (parameterType.isPrimitive() || parametersToClone[i] instanceof Number || parametersToClone[i] instanceof Boolean)
         {
            clone[i] = parametersToClone[i];
         }
         else if (DenseMatrix64F.class.equals(parameterType))
         {
            clone[i] = new DenseMatrix64F((DenseMatrix64F) parametersToClone[i]);
         }
         else if (float[].class.equals(parameterType))
         {
            float[] arrayToClone = (float[]) parametersToClone[i];
            clone[i] = new float[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (double[].class.equals(parameterType))
         {
            double[] arrayToClone = (double[]) parametersToClone[i];
            clone[i] = new double[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (int[].class.equals(parameterType))
         {
            int[] arrayToClone = (int[]) parametersToClone[i];
            clone[i] = new int[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (isVertexSupplier(parameterType))
         {
            clone[i] = parametersToClone[i];
         }
         else
         {
            try
            {
               if (parameterType.isArray())
               {
                  clone[i] = clone((Object[]) parametersToClone[i]);
               }
               else
               {
                  clone[i] = newInstanceOf(parameterType);
                  Method setter = parameterType.getMethod("set", parameterType);
                  setter.invoke(clone[i], parametersToClone[i]);
               }
            }
            catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               throw new RuntimeException("Unhandled type: " + parameterType.getSimpleName(), e);
            }
         }
      }

      return clone;
   }

   private static boolean isVertexSupplier(Class<?> classToTest)
   {
      boolean implementSupplier = Stream.of(Vertex2DSupplier.class, Vertex3DSupplier.class, FrameVertex2DSupplier.class, FrameVertex3DSupplier.class)
                                        .anyMatch(supplierType -> supplierType.isAssignableFrom(classToTest));
      return implementSupplier && !ConvexPolygon2DReadOnly.class.isAssignableFrom(classToTest);
   }

   private static Object[] instantiateParameterTypes(ReferenceFrame frame, Class<?>[] parameterTypes)
   {
      Object[] parameters = new Object[parameterTypes.length];

      for (int i = 0; i < parameterTypes.length; i++)
      {
         Class<?> parameterType = parameterTypes[i];
         if (parameterType.isArray() && !parameterType.getComponentType().isPrimitive())
         {
            Class<?> componentType = parameterType.getComponentType();
            Object[] array = (Object[]) Array.newInstance(componentType, random.nextInt(15));
            for (int j = 0; j < array.length; j++)
            {
               array[j] = instantiateParameterType(frame, componentType);
               if (array[j] == null)
                  return null;
            }
            parameters[i] = array;
         }
         else
         {
            parameters[i] = instantiateParameterType(frame, parameterType);
            if (parameters[i] == null)
               return null;
         }
      }
      return parameters;
   }

   private static Object instantiateParameterType(ReferenceFrame frame, Class<?> parameterType)
   {
      Object object = createFrameObject(parameterType, frame);
      if (object != null)
         return object;
      object = createFramelessObject(parameterType);
      if (object != null)
         return object;
      return newInstanceOf(parameterType);
   }

   private static Object createFramelessObject(Class<?> type)
   {
      GenericTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, GenericTypeBuilder> entry : framelessTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(random);
   }

   private static Object createFrameObject(Class<?> type, ReferenceFrame referenceFrame)
   {
      RandomFrameTypeBuilder<?> builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, RandomFrameTypeBuilder<?>> entry : frameTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(random, referenceFrame);
   }

   private static Object newInstanceOf(Class<?> type)
   {
      if (type.isPrimitive())
      {
         if (type.equals(Boolean.TYPE))
            return random.nextBoolean();
         else if (type.equals(Integer.TYPE) || type.equals(Character.TYPE) || type.equals(Long.TYPE))
            return random.nextInt(1000) - 500;
         else if (type.equals(Float.TYPE) || type.equals(Double.TYPE))
            return EuclidCoreRandomTools.nextDouble(random, 10.0);
         else
            return 0;
      }

      if (Transform.class.equals(type))
         return EuclidCoreRandomTools.nextAffineTransform(random);
      if (RigidBodyTransformReadOnly.class.isAssignableFrom(type))
         return EuclidCoreRandomTools.nextRigidBodyTransform(random);

      if (DenseMatrix64F.class.equals(type))
      {
         return RandomMatrices.createRandom(20, 20, random);
      }

      if (float[].class.equals(type))
      {
         float[] ret = new float[20];
         for (int i = 0; i < ret.length; i++)
            ret[i] = random.nextFloat();
         return ret;
      }

      if (double[].class.equals(type))
      {
         double[] ret = new double[20];
         for (int i = 0; i < ret.length; i++)
            ret[i] = random.nextDouble();
         return ret;
      }

      if (Collection.class.equals(type))
      {
         return null;
      }

      if (Object.class.equals(type))
      {
         return null;
      }

      try
      {
         return type.getConstructor().newInstance();
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
            | SecurityException e)
      {
         throw new RuntimeException("Could not instantiate an object of the type: " + type.getSimpleName() + " " + type);
      }
   }

   /**
    * Implement this interface to create builders for frame types such as {@code FramePoint2D}.
    * <p>
    * The frame objects created using this builder should contain random values changing from one
    * object to the next.
    * </p>
    *
    * @author Sylvain Bertrand
    * @param <T> the type this builder can instantiate.
    */
   public static interface RandomFrameTypeBuilder<T>
   {
      /**
       * Creates a new instance of the frame type.
       * <p>
       * The frame objects created using this builder should contain random values changing from one
       * object to the next.
       * </p>
       *
       * @param referenceFrame the reference frame in which the returned frame object should be expressed
       *                       in.
       * @return the next random frame object.
       */
      T newInstance(Random random, ReferenceFrame referenceFrame);
   }

   /**
    * Implement this interface to create builders for frame types such as {@code FrameQuaternion}.
    * <p>
    * The frame objects created using this builder should be initialized using the given reference
    * frame and frameless object.
    * </p>
    *
    * @author Sylvain Bertrand
    * @param <T> the type this builder can instantiate.
    */
   public static interface FrameTypeBuilder<T extends ReferenceFrameHolder>
   {
      /**
       * Creates a new instance of the frame type.
       * <p>
       * The frame objects created using this builder should be initialized using the given reference
       * frame and frameless object.
       * </p>
       *
       * @param referenceFrame  the reference frame in which the returned frame object should be expressed
       *                        in.
       * @param framelessObject the frameless object to use for initializing the values of the new frame
       *                        object.
       * @return the new frame object.
       */
      T newInstance(ReferenceFrame referenceFrame, Object framelessObject);
   }

   /**
    * Implement this interface to create builders for any type.
    * <p>
    * The objects created using this builder should contain random values changing from one object to
    * the next.
    * </p>
    *
    * @author Sylvain Bertrand
    */
   public static interface GenericTypeBuilder
   {
      /**
       * Creates a new instance of the same object initialized with random values.
       *
       * @return the next object.
       */
      Object newInstance(Random random);
   }

   private static Class<?> searchSuperInterfaceFromSimpleName(String name, Class<?> typeToStartFrom)
   {
      for (Class<?> superInterface : typeToStartFrom.getInterfaces())
      {
         if (superInterface.getSimpleName().equals(name))
         {
            return superInterface;
         }
      }

      for (Class<?> superInterface : typeToStartFrom.getInterfaces())
      {
         Class<?> thoroughSearchResult = searchSuperInterfaceFromSimpleName(name, superInterface);
         if (thoroughSearchResult != null)
            return thoroughSearchResult;
      }
      return null;
   }

   private static GenericTypeBuilder searchFramelessGenerator(Class<?> framelessMutableType)
   {
      List<Method> searchResult = new ArrayList<>();

      List<Method> allMethods = framelessRandomGeneratorLibrary.stream().flatMap(generatorClass -> Stream.of(generatorClass.getMethods()))
                                                               .collect(Collectors.toList());

      for (Method method : allMethods)
      {
         if (!Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
            continue;

         if (method.getParameterCount() != 1)
            continue;

         if (method.getParameterTypes()[0] != Random.class)
            continue;

         if (!framelessMutableType.isAssignableFrom(method.getReturnType()))
            continue;

         searchResult.add(method);
      }

      int nameLength = Integer.MAX_VALUE;
      Method randomGenerator = null;

      for (Method method : searchResult)
      {
         if (method.getName().length() < nameLength)
         {
            nameLength = method.getName().length();
            randomGenerator = method;
         }
      }
      System.out.println("Random generator for " + framelessMutableType.getSimpleName() + ", " + getMethodSimpleName(randomGenerator));

      Objects.requireNonNull(randomGenerator);
      final Method finalRandomGenerator = randomGenerator;

      return random ->
      {
         try
         {
            return finalRandomGenerator.invoke(null, random);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      };
   }

   private static RandomFrameTypeBuilder<?> searchFrameGenerator(Class<?> mutableFrameMutableType)
   {
      List<Method> searchResult = new ArrayList<>();

      List<Method> allMethods = frameRandomGeneratorLibrary.stream().flatMap(generatorClass -> Stream.of(generatorClass.getMethods()))
                                                           .collect(Collectors.toList());

      for (Method method : allMethods)
      {
         if (!Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
            continue;

         if (method.getParameterCount() != 2)
            continue;

         if (method.getParameterTypes()[0] != Random.class || method.getParameterTypes()[1] != ReferenceFrame.class)
            continue;

         if (!mutableFrameMutableType.isAssignableFrom(method.getReturnType()))
            continue;

         searchResult.add(method);
      }

      int nameLength = Integer.MAX_VALUE;
      Method randomGenerator = null;

      for (Method method : searchResult)
      {
         if (method.getName().length() < nameLength)
         {
            nameLength = method.getName().length();
            randomGenerator = method;
         }
      }

      Objects.requireNonNull(randomGenerator);

      System.out.println("Random generator for " + mutableFrameMutableType.getSimpleName() + ", " + getMethodSimpleName(randomGenerator));

      final Method finalRandomGenerator = randomGenerator;

      return (random, frame) ->
      {
         try
         {
            return finalRandomGenerator.invoke(null, random, frame);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      };
   }
}
