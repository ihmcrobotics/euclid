package us.ihmc.euclid.referenceFrame.api;

import static us.ihmc.euclid.referenceFrame.api.MethodSignature.getMethodSimpleName;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;
import java.util.Map.Entry;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
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
   private final static Random random = new Random(345345);
   private final static double epsilon = 1.0e-12;

   private final static Set<Class<?>> framelessTypesWithoutFrameEquivalent = new HashSet<>();
   private final static Map<Class<?>, Class<?>> framelessTypesToFrameTypesTable = new HashMap<>();
   private final static Set<Class<?>> frameReadOnlyTypes = new HashSet<>();
   private final static Set<Class<?>> fixedFrameMutableTypes = new HashSet<>();
   private final static Set<Class<?>> mutableFrameMutableTypes = new HashSet<>();

   static
   {
      registerEuclidFrameTypes();
   }

   private static void registerEuclidFrameTypes()
   {
      ReflectionBasedBuilders.registerRandomGeneratorClasses(EuclidCoreRandomTools.class, EuclidGeometryRandomTools.class, EuclidFrameRandomTools.class);

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
      registerFramelessType(RotationScaleMatrix.class, RotationScaleMatrixReadOnly.class);
   }

   private final static Set<Class<?>> acceptableExceptions = new HashSet<>();
   static
   {
      acceptableExceptions.add(BoundingBoxException.class);
      acceptableExceptions.add(IllegalArgumentException.class);
      acceptableExceptions.add(RuntimeException.class);
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

      Objects.requireNonNull(framelessReadOnlyType);
      framelessTypesWithoutFrameEquivalent.addAll(Arrays.asList(framelessMutableType, framelessReadOnlyType));
   }

   public static void registerFramelessType(Class<?> framelessMutableType, Class<?> framelessReadOnlyType)
   {
      framelessTypesWithoutFrameEquivalent.addAll(Arrays.asList(framelessMutableType, framelessReadOnlyType));
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

      Objects.requireNonNull(fixedFrameMutableType);
      Objects.requireNonNull(frameReadOnlyType);
      Objects.requireNonNull(framelessMutableType);
      Objects.requireNonNull(framelessReadOnlyType);

      framelessTypesToFrameTypesTable.put(framelessReadOnlyType, frameReadOnlyType);
      framelessTypesToFrameTypesTable.put(framelessMutableType, fixedFrameMutableType);
      frameReadOnlyTypes.add(frameReadOnlyType);
      fixedFrameMutableTypes.add(fixedFrameMutableType);
      mutableFrameMutableTypes.add(mutableFrameMutableType);
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

      framelessTypesToFrameTypesTable.put(framelessReadOnlyType, frameReadOnlyType);
      frameReadOnlyTypes.add(frameReadOnlyType);
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

   public static Predicate<Method> methodFilterFromSignature(Collection<MethodSignature> signaturesToIgnore)
   {
      List<Predicate<Method>> filters = signaturesToIgnore.stream().map(EuclidFrameAPITester::methodFilterFromSignature).collect(Collectors.toList());
      return method -> filters.stream().allMatch(filter -> filter.test(method));
   }

   public static Predicate<Method> methodFilterFromSignature(MethodSignature signatureToIgnore)
   {
      return method ->
      {
         if (!signatureToIgnore.getName().equals(method.getName()))
            return true;
         if (Arrays.equals(method.getParameterTypes(), signatureToIgnore.toParameterTypeArray()))
            return false;
         else
            return true;
      };
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
      Predicate<Method> filter = framelessMethodFilter.and(atLeastNFramelessParameters(minNumberOfFramelessArguments));
      List<MethodSignature> framelessSignatures = Stream.of(typeWithFramelessMethods.getMethods()).filter(filter).map(MethodSignature::new)
                                                        .collect(Collectors.toList());

      for (MethodSignature framelessSignature : framelessSignatures)
      {
         // Creating all the expected combinations
         List<MethodSignature> expectedMethodSignatures = createExpectedMethodSignaturesWithFrameArgument(framelessSignature, assertAllCombinations);

         for (MethodSignature expectedMethodSignature : expectedMethodSignatures)
         {
            assertMethodOverloadedWithSpecificSignature(typeWithFrameMethods, typeWithFramelessMethods, framelessSignature, expectedMethodSignature);
         }
      }
   }

   private static void assertMethodOverloadedWithSpecificSignature(Class<?> typeWithOverloadingMethods, Class<?> typeWithOriginalMethod,
                                                                   MethodSignature originalSignature, MethodSignature overloadingSignature)
         throws SecurityException
   {
      try
      {
         Method overloadingMethod = typeWithOverloadingMethods.getMethod(originalSignature.getName(), overloadingSignature.toParameterTypeArray());
         Class<?> originalReturnType = originalSignature.getReturnType();
         Class<?> overloadingReturnType = overloadingMethod.getReturnType();

         { // Assert the return type is proper
            if (originalReturnType == null && overloadingReturnType != null)
            {
               String message = "Inconsistency found in the return type.";
               message += "\nOriginal method: " + originalSignature.getMethodSimpleName();
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
         throw new AssertionError("The original method in " + typeWithOriginalMethod.getSimpleName() + ":\n" + originalSignature.getMethodSimpleName()
               + "\nis not properly overloaded, expected to find in " + typeWithOverloadingMethods.getSimpleName() + ":\n"
               + overloadingSignature.getMethodSimpleName());
      }
   }

   /**
    * Convention is that for a setter like {@code FramelessType.set(Point2DReadOnly, Tuple3DReadOnly)}
    * the following setters should be declared in the frame type:
    * <ul>
    * <li>{@code FrameType.setMatchingFrame(ReferenceFrame, Point2DReadOnly, Tuple3DReadOnly)}
    * <li>{@code FrameType.setMatchingFrame(FramePoint2DReadOnly, FrameTuple3DReadOnly)}
    * </ul>
    *
    * @param typeWithFrameMethods
    * @param typeWithFramelessMethods
    * @param framelessMethodFilter
    */
   public static void assertAPIDeclareMatchingFrameSetters(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods,
                                                           Predicate<Method> framelessMethodFilter)
         throws SecurityException
   {
      Predicate<Method> filter = framelessMethodFilter.and(atLeastNFramelessParameters(1)).and(m -> m.getName().equals("set"));
      List<MethodSignature> framelessSignatures = Stream.of(typeWithFramelessMethods.getMethods()).filter(filter).map(MethodSignature::new)
                                                        .collect(Collectors.toList());

      for (MethodSignature framelessSetterSignature : framelessSignatures)
      {
         List<MethodSignature> expectedSetMatchingFrameSignatures = createExpectedSetMatchingFrameSignatures(framelessSetterSignature);

         for (MethodSignature expectedSetMatchingFrameSignature : expectedSetMatchingFrameSignatures)
         {
            try
            {
               Method setMatchingFrameMethod = typeWithFrameMethods.getMethod(expectedSetMatchingFrameSignature.getName(),
                                                                              expectedSetMatchingFrameSignature.toParameterTypeArray());

               Class<?> originalReturnType = framelessSetterSignature.getReturnType();
               Class<?> overloadingReturnType = setMatchingFrameMethod.getReturnType();

               { // Assert the return type is proper
                  if (originalReturnType == null && overloadingReturnType != null)
                  {
                     String message = "Inconsistency found in the return type.";
                     message += "\nOriginal setter: " + framelessSetterSignature.getMethodSimpleName();
                     message += "\nCorresponding setMatchingFrame: " + getMethodSimpleName(setMatchingFrameMethod);
                     message += "\nOriginal type declaring method: " + typeWithFramelessMethods.getSimpleName();
                     message += "\nType declaring setMatchingFrame: " + typeWithFrameMethods.getSimpleName();
                     throw new AssertionError(message);
                  }

                  if (overloadingReturnType.equals(originalReturnType))
                     return;

                  if (overloadingReturnType.isAssignableFrom(findCorrespondingFrameType(originalReturnType)))
                     throw new AssertionError("Unexpected return type: expected: " + findCorrespondingFrameType(originalReturnType).getSimpleName()
                           + ", actual: " + overloadingReturnType.getSimpleName());
               }
            }
            catch (NoSuchMethodException e)
            {
               throw new AssertionError("Could not find setMatchingFrame correspond to the original setter in " + typeWithFramelessMethods.getSimpleName()
                     + ":\n" + framelessSetterSignature.getMethodSimpleName() + "\nExpected to find in " + typeWithFrameMethods.getSimpleName() + ":\n"
                     + expectedSetMatchingFrameSignature.getMethodSimpleName());
            }
         }
      }
   }

   private static List<MethodSignature> createExpectedSetMatchingFrameSignatures(MethodSignature framelessSetterSignature)
   {
      assert framelessSetterSignature.getName().equals("set");

      List<MethodSignature> signatures = new ArrayList<>();
      // setMatchingFrame(ReferenceFrame, FramelessType(s))
      MethodSignature expectedSetMatchingFrameSignature = new MethodSignature(framelessSetterSignature);
      expectedSetMatchingFrameSignature.setName("setMatchingFrame");
      expectedSetMatchingFrameSignature.addParameterType(0, ReferenceFrame.class);
      signatures.add(expectedSetMatchingFrameSignature);

      // setMatchingFrame(FrameType(s))
      expectedSetMatchingFrameSignature = new MethodSignature(framelessSetterSignature);
      expectedSetMatchingFrameSignature.setName("setMatchingFrame");

      for (int i = 0; i < expectedSetMatchingFrameSignature.getParameterCount(); i++)
      {
         Class<?> parameterType = expectedSetMatchingFrameSignature.getParameterType(i);
         if (isFramelessTypeWithFrameEquivalent(parameterType))
            expectedSetMatchingFrameSignature.setParameterType(i, findCorrespondingFrameType(parameterType));
      }
      signatures.add(expectedSetMatchingFrameSignature);
      return signatures;
   }

   public static void assertSetMatchingFramePreserveFunctionality(RandomFrameTypeBuilder frameTypeBuilder)
   {
      assertSetMatchingFramePreserveFunctionality(frameTypeBuilder, m -> true);
   }

   public static void assertSetMatchingFramePreserveFunctionality(RandomFrameTypeBuilder frameTypeBuilder, Predicate<Method> methodFilter)
   {
      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(random, worldFrame).getClass();

      Predicate<Method> filter = methodFilter.and(m -> m.getName().equals("setMatchingFrame"));
      List<Method> frameMethods = Stream.of(frameType.getMethods()).filter(filter).collect(Collectors.toList());

      for (Method matchingFrameMethod : frameMethods)
      {
         try
         {
            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

            Method setterMethod = findCorrespondingSetterToSetMatchingFrame(frameType, matchingFrameMethod);

            Object[] matchingFrameMethodParameters = ReflectionBasedBuilders.next(random, frameA, matchingFrameMethod.getParameterTypes());
            Object[] setterMethodParameters = ReflectionBasedBuilders.clone(matchingFrameMethodParameters);
            boolean isLastParameterToCheck2DTransform = is2DType(frameType)
                  && matchingFrameMethod.getParameterTypes()[matchingFrameMethod.getParameterCount() - 1] == boolean.class;

            if (isLastParameterToCheck2DTransform)
            { // Last argument is "boolean checkIfTransformInXYPlane"
               setterMethodParameters = Arrays.copyOfRange(setterMethodParameters, 0, setterMethodParameters.length - 1);
            }

            ReferenceFrameHolder matchingFrameObject = frameTypeBuilder.newInstance(random, frameB);
            ReferenceFrameHolder setterObject = frameTypeBuilder.newInstance(random, frameA);

            Throwable expectedException = null;
            Object setterMethodReturnObject = null;
            Object matchingFrameMethodReturnObject = null;

            try
            {
               if (isLastParameterToCheck2DTransform)
               {
                  setterMethodReturnObject = invokeMethod(setterObject, setterMethod, setterMethodParameters);
                  Method applyTransformMethod = frameType.getMethod("applyTransform", Transform.class, boolean.class);
                  invokeMethod(setterObject,
                               applyTransformMethod,
                               frameA.getTransformToDesiredFrame(frameB),
                               matchingFrameMethodParameters[matchingFrameMethodParameters.length - 1]);
                  ((FrameChangeable) setterObject).setReferenceFrame(frameB);
               }
               else if (is2DType(frameType) && Stream.of(setterMethodParameters).map(Object::getClass).allMatch(EuclidFrameAPITester::is3DType))
               { // The transformation should be done on the arguments not the holder.
                  setterObject = frameTypeBuilder.newInstance(random, frameB);

                  Object[] localSetterMethodParameters = ReflectionBasedBuilders.clone(setterMethodParameters);

                  for (int paramIndex = 0; paramIndex < localSetterMethodParameters.length; paramIndex++)
                  {
                     Object setterMethodParameter = localSetterMethodParameters[paramIndex];

                     if (setterMethodParameter instanceof FrameVertex3DSupplier)
                     {
                        FrameVertex3DSupplier asSupplier = (FrameVertex3DSupplier) setterMethodParameter;
                        List<FramePoint3D> vertices = new ArrayList<>();
                        for (int vertexIndex = 0; vertexIndex < asSupplier.getNumberOfVertices(); vertexIndex++)
                           vertices.add(new FramePoint3D(asSupplier.getVertex(vertexIndex)));
                        vertices.forEach(v -> v.changeFrame(frameB));
                        localSetterMethodParameters[paramIndex] = FrameVertex3DSupplier.asFrameVertex3DSupplier(vertices);
                     }
                     else
                     {
                        ((FrameChangeable) setterMethodParameter).changeFrame(frameB);
                     }
                  }
                  setterMethodReturnObject = invokeMethod(setterObject, setterMethod, localSetterMethodParameters);
               }
               else
               {
                  setterMethodReturnObject = invokeMethod(setterObject, setterMethod, setterMethodParameters);
                  ((FrameChangeable) setterObject).changeFrame(frameB);
               }
            }
            catch (Throwable e)
            {
               expectedException = e;
            }

            try
            {
               matchingFrameMethodReturnObject = invokeMethod(matchingFrameObject, matchingFrameMethod, matchingFrameMethodParameters);
            }
            catch (Throwable e)
            {
               if (expectedException == null || e.getClass() != expectedException.getClass())
               {
                  reportInconsistentException(matchingFrameMethod, setterMethod, expectedException, e);
               }
               else
               {
                  continue;
               }
            }

            for (int i = 0; i < setterMethodParameters.length; i++)
            {
               Object setterParameter = setterMethodParameters[i];
               Object matchingFrameParameter = matchingFrameMethodParameters[i];

               if (!ReflectionBasedComparer.epsilonEquals(setterParameter, matchingFrameParameter, epsilon))
                  reportInconsistentArguments(matchingFrameMethod,
                                              setterMethod,
                                              matchingFrameMethodParameters,
                                              setterMethodParameters,
                                              setterParameter,
                                              matchingFrameParameter);
            }

            if (!ReflectionBasedComparer.epsilonEquals(setterMethodReturnObject, matchingFrameMethodReturnObject, epsilon))
               reportInconsistentReturnedType(matchingFrameMethod, setterMethod, setterMethodReturnObject, matchingFrameMethodReturnObject);

            if (!ReflectionBasedComparer.epsilonEquals(setterObject, matchingFrameObject, epsilon))
               reportInconsistentObject(matchingFrameMethod, setterObject, matchingFrameObject, setterMethod);
         }
         catch (RuntimeException e)
         {
            System.err.println("Problem when evaluating the method: " + getMethodSimpleName(matchingFrameMethod));
            throw e;
         }
      }
   }

   private static Method findCorrespondingSetterToSetMatchingFrame(Class<?> frameType, Method setMatchingFrameMethod)
   {
      MethodSignature frameSetterSignature = new MethodSignature(setMatchingFrameMethod);
      frameSetterSignature.setName("set");

      Class<?> lastParameter = setMatchingFrameMethod.getParameterTypes()[setMatchingFrameMethod.getParameterCount() - 1];

      if (lastParameter == boolean.class && is2DType(frameType))
      {
         frameSetterSignature.removeParameterType(setMatchingFrameMethod.getParameterCount() - 1);
      }

      try
      {
         return frameType.getMethod(frameSetterSignature.getName(), frameSetterSignature.toParameterTypeArray());
      }
      catch (NoSuchMethodException e)
      {
         throw new AssertionError("Could not find the frameless setter that corresponds to :\n" + getMethodSimpleName(setMatchingFrameMethod) + ", declared in "
               + setMatchingFrameMethod.getDeclaringClass().getSimpleName() + "\nExpected to find in " + frameType.getSimpleName() + ":\n"
               + frameSetterSignature.getMethodSimpleName());
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
    * @param numberOfIterations               number of iterations to perform for each method.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, int numberOfIterations) throws Throwable
   {
      assertStaticMethodsCheckReferenceFrame(typeDeclaringStaticMethodsToTest, m -> true, numberOfIterations);
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
    * @param numberOfIterations               number of iterations to perform for each method.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, Predicate<Method> methodFilter, int numberOfIterations)
         throws Throwable
   {
      Predicate<Method> filter = methodFilter.and(atLeastNFrameParameters(2)).and(m -> Modifier.isStatic(m.getModifiers()))
                                             .and(m -> Modifier.isPublic(m.getModifiers()));
      List<Method> frameMethods = Stream.of(typeDeclaringStaticMethodsToTest.getMethods()).filter(filter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < numberOfIterations; iteration++)
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
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
                     parameters[j] = ReflectionBasedBuilders.next(random, frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = ReflectionBasedBuilders.next(random, frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeStaticMethod(frameMethod, parameters);
                  failToThrowReferenceFrameMismatchException(typeDeclaringStaticMethodsToTest, frameMethod, parameters);
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
                  parameters[i] = ReflectionBasedBuilders.next(random, frameB, parameterType);
               else
                  parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
                     failToChangeParameterFrame(typeDeclaringStaticMethodsToTest, frameMethod, parameters, i);
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
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               failToSetResultFrame(typeDeclaringStaticMethodsToTest, frameMethod, parameters, result);
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
    * @param frameTypeBuilder   builder used to generate an instance of the type to be tested.
    * @param methodFilter       custom filter used on the methods. The assertions are performed on the
    *                           methods for which {@code methodFilter.test(method)} returns
    *                           {@code true}.
    * @param numberOfIterations number of iterations to perform for each method.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertMethodsOfReferenceFrameHolderCheckReferenceFrame(RandomFrameTypeBuilder frameTypeBuilder, Predicate<Method> methodFilter,
                                                                             int numberOfIterations)
         throws Throwable
   {
      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(random, worldFrame).getClass();

      Predicate<Method> filter = methodFilter.and(m -> Modifier.isPublic(m.getModifiers())).and(atLeastNFrameParameters(1));

      List<Method> frameMethods = Stream.of(frameType.getMethods()).filter(filter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < numberOfIterations; iteration++)
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
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
                     parameters[j] = ReflectionBasedBuilders.next(random, frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = ReflectionBasedBuilders.next(random, frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeMethod(frameObject, frameMethod, parameters);
                  failToThrowReferenceFrameMismatchException(frameType, frameMethod, parameters);
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
                  parameters[i] = ReflectionBasedBuilders.next(random, frameB, parameterType);
               else
                  parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
                     failToChangeParameterFrame(frameType, frameMethod, parameters, i);
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
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               failToSetResultFrame(frameType, frameMethod, parameters, result);
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
    * @param numberOfIterations         number of iterations to perform for each method.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods, int numberOfIterations)
   {
      assertStaticMethodsPreserveFunctionality(typeWithFrameMethodsToTest, typeWithFramelessMethods, m -> true, numberOfIterations);
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
    * @param numberOfIterations         number of iterations to perform for each method.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods,
                                                               Predicate<Method> methodFilter, int numberOfIterations)
   {
      List<Method> frameMethods = Stream.of(typeWithFrameMethodsToTest.getMethods()).filter(methodFilter).collect(Collectors.toList());

      for (Method frameMethod : frameMethods)
      {
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

         for (int iteration = 0; iteration < numberOfIterations; iteration++)
         {
            try
            {
               Method framelessMethod = typeWithFramelessMethods.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = ReflectionBasedBuilders.next(random, worldFrame, frameMethodParameterTypes);

               String message = "Could not instantiate the parameters for the method: " + getMethodSimpleName(frameMethod) + ". The method is not tested.";
               Objects.requireNonNull(frameMethodParameters, message);

               Object[] framelessMethodParameters = ReflectionBasedBuilders.clone(frameMethodParameters);
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
                  if (expectedException == null || e.getClass() != expectedException.getClass())
                  {
                     reportInconsistentException(frameMethod, framelessMethod, expectedException, e);
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

                  if (!ReflectionBasedComparer.epsilonEquals(framelessParameter, frameParameter, epsilon))
                     reportInconsistentArguments(frameMethod,
                                                 framelessMethod,
                                                 frameMethodParameters,
                                                 framelessMethodParameters,
                                                 framelessParameter,
                                                 frameParameter);
               }

               if (!ReflectionBasedComparer.epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
                  reportInconsistentReturnedType(frameMethod, framelessMethod, framelessMethodReturnObject, frameMethodReturnObject);
            }
            catch (NoSuchMethodException e)
            {
               debugNoSuchMethodException(typeWithFrameMethodsToTest, typeWithFramelessMethods, frameMethod, framelessMethodParameterTypes);
            }
            catch (SecurityException e)
            {
               debugSecurityException(typeWithFramelessMethods, frameMethodName, framelessMethodParameterTypes);
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
    * @param frameTypeCopier      the builder for creating instances of the frame object to test.
    * @param framelessTypeBuilber the builder for creating instances of the corresponding frameless
    *                             objects.
    * @param methodFilter         custom filter used on the methods. The assertions are performed on
    *                             the methods for which {@code methodFilter.test(method)} returns
    *                             {@code true}.
    * @param numberOfIterations   number of iterations to perform for each method.
    */
   public static void assertFrameMethodsOfFrameHolderPreserveFunctionality(FrameTypeCopier frameTypeCopier, RandomFramelessTypeBuilder framelessTypeBuilber,
                                                                           Predicate<Method> methodFilter, int numberOfIterations)
   {

      Class<? extends ReferenceFrameHolder> frameTypeToTest = frameTypeCopier.newInstance(worldFrame, framelessTypeBuilber.newInstance(random)).getClass();
      Class<? extends Object> framelessType = framelessTypeBuilber.newInstance(random).getClass();

      List<Method> frameMethods = Stream.of(frameTypeToTest.getMethods()).filter(methodFilter).collect(Collectors.toList());

      for (Method frameMethod : frameMethods)
      {
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

         for (int iteration = 0; iteration < numberOfIterations; iteration++)
         {
            Object framelessObject = framelessTypeBuilber.newInstance(random);
            ReferenceFrameHolder frameObject = frameTypeCopier.newInstance(worldFrame, framelessObject);

            try
            {
               Method framelessMethod = framelessType.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = ReflectionBasedBuilders.next(random, worldFrame, frameMethodParameterTypes);

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

               Object[] framelessMethodParameters = ReflectionBasedBuilders.clone(frameMethodParameters);
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
                     reportInconsistentException(frameMethod, framelessMethod, expectedException, e);
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

                  if (!ReflectionBasedComparer.epsilonEquals(framelessParameter, frameParameter, epsilon))
                     reportInconsistentArguments(frameMethod,
                                                 framelessMethod,
                                                 frameMethodParameters,
                                                 framelessMethodParameters,
                                                 framelessParameter,
                                                 frameParameter);
               }

               if (!ReflectionBasedComparer.epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
                  reportInconsistentReturnedType(frameMethod, framelessMethod, framelessMethodReturnObject, frameMethodReturnObject);

               if (!ReflectionBasedComparer.epsilonEquals(framelessObject, frameObject, epsilon))
                  reportInconsistentObject(frameMethod, framelessObject, frameObject, framelessMethod);
            }
            catch (NoSuchMethodException e)
            {
               debugNoSuchMethodException(frameTypeToTest, framelessType, frameMethod, framelessMethodParameterTypes);
            }
            catch (SecurityException e)
            {
               debugSecurityException(framelessType, frameMethodName, framelessMethodParameterTypes);
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

   private static void failToSetResultFrame(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters, Object result) throws AssertionError
   {
      String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not set the frame of the result.";
      message += "\nType being tested: " + typeDeclaringMethod.getSimpleName();
      message += "\nArguments used: " + Arrays.toString(parameters);
      message += "\nArgument types: " + getArgumentTypeString(parameters);
      message += "\nResult: " + result;
      throw new AssertionError(message);
   }

   private static void failToChangeParameterFrame(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters, int parameterIndex)
         throws AssertionError
   {
      String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + (parameterIndex + 1) + "th parameter.";
      message += "\nType being tested: " + typeDeclaringMethod.getSimpleName();
      message += "\nArguments used: " + Arrays.toString(parameters);
      message += "\nArgument types: " + getArgumentTypeString(parameters);
      throw new AssertionError(message);
   }

   private static void failToThrowReferenceFrameMismatchException(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters) throws AssertionError
   {
      String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
      message += "\nType being tested: " + typeDeclaringMethod.getSimpleName();
      message += "\nMethod: " + getMethodSimpleName(frameMethod);
      message += "\nArguments used: " + Arrays.toString(parameters);
      message += "\nArgument types: " + getArgumentTypeString(parameters);
      throw new AssertionError(message);
   }

   private static void debugSecurityException(Class<?> typeWithFramelessMethods, String frameMethodName, Class<?>[] framelessMethodParameterTypes)
   {
      if (DEBUG)
      {
         String message = "";
         message += "-------------------------------------------------------------------";
         message += "\nUnable to access method with name: " + frameMethodName + " and argument types: " + getSimpleNames(framelessMethodParameterTypes);
         message += "\nin type: " + typeWithFramelessMethods.getSimpleName();
         message += "\n-------------------------------------------------------------------";
         System.err.println(message);
      }
   }

   private static void debugNoSuchMethodException(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods, Method frameMethod,
                                                  Class<?>[] framelessMethodParameterTypes)
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

   private static void reportInconsistentObject(Method frameMethod, Object framelessObject, ReferenceFrameHolder frameObject, Method framelessMethod)
         throws AssertionError
   {
      String message = "";
      message += "Detected a frame method inconsistent with its original frameless method.";
      message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
      message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
      message += "\nFrame object after method call:" + frameObject;
      message += "\nFrameless object after method call:" + framelessObject;
      throw new AssertionError(message);
   }

   private static void reportInconsistentException(Method frameMethod, Method framelessMethod, Throwable expectedException, Throwable e) throws AssertionError
   {
      String message = "";
      message += "The method: " + getMethodSimpleName(frameMethod);
      message += "\ndid not throw the same exception as the original method: " + getMethodSimpleName(framelessMethod);
      message += "\nExpected exception class: " + (expectedException == null ? "none" : expectedException.getClass().getSimpleName());
      message += "\nActual exception class: " + e.getClass().getSimpleName();
      throw new AssertionError(message);
   }

   private static void reportInconsistentArguments(Method frameMethod, Method framelessMethod, Object[] frameMethodParameters,
                                                   Object[] framelessMethodParameters, Object framelessParameter, Object frameParameter)
         throws AssertionError
   {
      String message = "";
      message += "Detected a frame method inconsistent with its original frameless method.";
      message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
      message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
      message += "\nFrame arguments after call:\n" + Arrays.toString(frameMethodParameters);
      message += "\nFrameless arguments after call:\n"
            + EuclidCoreIOTools.getCollectionString("[", "]", ", ", Arrays.asList(framelessMethodParameters), EuclidFrameAPITester::toStringAsFramelessObject);
      throw new AssertionError(message);
   }

   private static void reportInconsistentReturnedType(Method frameMethod, Method framelessMethod, Object framelessMethodReturnObject,
                                                      Object frameMethodReturnObject)
         throws AssertionError
   {
      String message = "";
      message += "Detected a frame method inconsistent with its original frameless method.";
      message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
      message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
      message += "\nFrame method returned:" + frameMethodReturnObject;
      message += "\nFrameless method returned:" + toStringAsFramelessObject(framelessMethodReturnObject);
      throw new AssertionError(message);
   }

   private static String toStringAsFramelessObject(Object frameObject)
   {
      if (isFrameObject(frameObject))
         return findCorrespondingFramelessType(frameObject.getClass()).cast(frameObject).toString();
      else
         return frameObject.toString();
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

   private static Object invokeMethod(Object methodHolder, Method frameMethod, Object... parameters) throws Throwable
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
      return EuclidCoreIOTools.getCollectionString(", ", Arrays.asList(arguments), o -> o.getClass().getSimpleName());
   }

   private static String getSimpleNames(Class<?>[] types)
   {
      String ret = Arrays.stream(types).map(t -> t.getSimpleName()).collect(Collectors.toList()).toString();
      return ret.substring(1, ret.length() - 1);
   }

   private static Predicate<Method> atLeastNFrameParameters(int minNumberOfFrameParameters)
   {
      return method -> countFrameParameters(method) >= minNumberOfFrameParameters;
   }

   private static int countFrameParameters(Method method)
   {
      return (int) Stream.of(method.getParameterTypes()).filter(EuclidFrameAPITester::isFrameType).count();
   }

   private static Predicate<Method> atLeastNFramelessParameters(int minNumberOfFramelessParameters)
   {
      return method -> countFramelessParameters(method) >= minNumberOfFramelessParameters;
   }

   private static int countFramelessParameters(Method method)
   {
      return (int) Stream.of(method.getParameterTypes()).filter(EuclidFrameAPITester::isFramelessType).count();
   }

   private static List<MethodSignature> createExpectedMethodSignaturesWithFrameArgument(MethodSignature framelessSignature, boolean createAllCombinations)
   {
      List<MethodSignature> expectedFrameSignatures = new ArrayList<>();

      if (!createAllCombinations)
      {
         MethodSignature combination = new MethodSignature(framelessSignature);

         for (int k = 0; k < combination.getParameterCount(); k++)
         {
            if (isFramelessTypeWithFrameEquivalent(combination.getParameterType(k)))
               combination.setParameterType(k, findCorrespondingFrameType(combination.getParameterType(k)));
         }
         expectedFrameSignatures.add(combination);
      }
      else
      {
         int numberOfArgumentsToOverload = (int) Arrays.stream(framelessSignature.toParameterTypeArray()).filter(t -> isFramelessTypeWithFrameEquivalent(t))
                                                       .count();
         int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToOverload);

         for (int i = 0; i < numberOfCombinations; i++)
         {
            MethodSignature combination = new MethodSignature(framelessSignature);
            int currentByte = 0;

            for (int k = 0; k < combination.getParameterCount(); k++)
            {
               if (isFramelessTypeWithFrameEquivalent(combination.getParameterType(k)))
               {
                  int mask = (int) Math.pow(2, currentByte);
                  if ((i & mask) != 0)
                     combination.setParameterType(k, findCorrespondingFrameType(combination.getParameterType(k)));
                  currentByte++;
               }
            }
            expectedFrameSignatures.add(combination);
         }

         // Remove the original method from the combinations
         expectedFrameSignatures = expectedFrameSignatures.stream().filter(signature -> !signature.equals(framelessSignature)).collect(Collectors.toList());
      }
      return expectedFrameSignatures;
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

   private static boolean isFramelessTypeWithFrameEquivalent(Class<?> framelessType)
   {
      if (framelessType.isArray())
         return isFramelessTypeWithFrameEquivalent(framelessType.getComponentType());
      return isFramelessType(framelessType) && !framelessTypesWithoutFrameEquivalent.contains(framelessType);
   }

   private static boolean is2DType(Class<?> type)
   {
      return findCorrespondingFramelessType(type).getSimpleName().contains("2D");
   }

   private static boolean is3DType(Class<?> type)
   {
      return findCorrespondingFramelessType(type).getSimpleName().contains("3D");
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
}
