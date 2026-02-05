#!/bin/bash
# 批量运行所有 scene19 配置文件
# 配置目录: generated_configs_scene19/main
# 特点：使用 Camera_03 和 gemini_scene_19 USD 文件

CONFIG_DIR="generated_configs_scene19/main"

echo "🚀 开始批量运行 scene19 配置文件..."
echo "📁 配置目录: $CONFIG_DIR"
echo "📷 相机配置: Camera_03"
echo "🎬 USD 路径: gemini_scene_19"
echo "================================================================"
echo ""

# 1. Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop
echo "📋 [1/30] 运行: Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop

echo ""

# 2. Basic_Methanolysis_of_an_Acetate_Ester
echo "📋 [2/30] 运行: Basic_Methanolysis_of_an_Acetate_Ester"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Basic_Methanolysis_of_an_Acetate_Ester

echo ""

# 3. Boc_Deprotection_of_Benzyl-methyl-piperidin-4-yl-a
echo "📋 [3/30] 运行: Boc_Deprotection_of_Benzyl-methyl-piperidin-4-yl-a"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Boc_Deprotection_of_Benzyl-methyl-piperidin-4-yl-a

echo ""

# 4. Boc_Deprotection_of_Tert-Butyl_Carbazate
echo "📋 [4/30] 运行: Boc_Deprotection_of_Tert-Butyl_Carbazate"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Boc_Deprotection_of_Tert-Butyl_Carbazate

echo ""

# 5. Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol
echo "📋 [5/30] 运行: Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol

echo ""

# 6. Deprotection_and_Cyclization_of_an_Indole_Derivati
echo "📋 [6/30] 运行: Deprotection_and_Cyclization_of_an_Indole_Derivati"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Deprotection_and_Cyclization_of_an_Indole_Derivati

echo ""

# 7. Deprotection_of_Pyrrolidone_Derivative_TFA_Method
echo "📋 [7/30] 运行: Deprotection_of_Pyrrolidone_Derivative_TFA_Method"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Deprotection_of_Pyrrolidone_Derivative_TFA_Method

echo ""

# 8. Fmoc_Deprotection_of_Peptide_Intermediate
echo "📋 [8/30] 运行: Fmoc_Deprotection_of_Peptide_Intermediate"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Fmoc_Deprotection_of_Peptide_Intermediate

echo ""

# 9. Hydrolysis_of_Ethyl_Crotonate_using_Lithium_Hydrox
echo "📋 [9/30] 运行: Hydrolysis_of_Ethyl_Crotonate_using_Lithium_Hydrox"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Hydrolysis_of_Ethyl_Crotonate_using_Lithium_Hydrox

echo ""

# 10. Hydrolysis_of_Ethyl_Ester_using_Lithium_Hydroxide
echo "📋 [10/30] 运行: Hydrolysis_of_Ethyl_Ester_using_Lithium_Hydroxide"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Hydrolysis_of_Ethyl_Ester_using_Lithium_Hydroxide

echo ""

# 11. Hydrolysis_of_Nitrile_to_Amide
echo "📋 [11/30] 运行: Hydrolysis_of_Nitrile_to_Amide"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Hydrolysis_of_Nitrile_to_Amide

echo ""

# 12. Knoevenagel_Condensation_Protocol
echo "📋 [12/30] 运行: Knoevenagel_Condensation_Protocol"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Knoevenagel_Condensation_Protocol

echo ""

# 13. Oxidation_of_Nicotine_to_Nicotinic_Acid
echo "📋 [13/30] 运行: Oxidation_of_Nicotine_to_Nicotinic_Acid"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Oxidation_of_Nicotine_to_Nicotinic_Acid

echo ""

# 14. Oxidation_of_Sulfide_to_Sulfoxide
echo "📋 [14/30] 运行: Oxidation_of_Sulfide_to_Sulfoxide"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Oxidation_of_Sulfide_to_Sulfoxide

echo ""

# 15. Preparation_of_13-Benzothiazol-6-sulfinic_Acid_Sod
echo "📋 [15/30] 运行: Preparation_of_13-Benzothiazol-6-sulfinic_Acid_Sod"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Preparation_of_13-Benzothiazol-6-sulfinic_Acid_Sod

echo ""

# 16. Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride_
echo "📋 [16/30] 运行: Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride_"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride_

echo ""

# 17. Preparation_of_Acid_Chloride_using_Thionyl_Chlorid
echo "📋 [17/30] 运行: Preparation_of_Acid_Chloride_using_Thionyl_Chlorid"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Preparation_of_Acid_Chloride_using_Thionyl_Chlorid

echo ""

# 18. Preparation_of_Chlorophenylisocyanate
echo "📋 [18/30] 运行: Preparation_of_Chlorophenylisocyanate"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Preparation_of_Chlorophenylisocyanate

echo ""

# 19. Preparation_of_Ethyl_4-chloromethyl-125-trimethylp
echo "📋 [19/30] 运行: Preparation_of_Ethyl_4-chloromethyl-125-trimethylp"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Preparation_of_Ethyl_4-chloromethyl-125-trimethylp

echo ""

# 20. Preparation_of_Phenylphosphonic_Dichloride
echo "📋 [20/30] 运行: Preparation_of_Phenylphosphonic_Dichloride"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Preparation_of_Phenylphosphonic_Dichloride

echo ""

# 21. Synthesis_of_3-4-acetyloxyphenylglutaric_anhydride
echo "📋 [21/30] 运行: Synthesis_of_3-4-acetyloxyphenylglutaric_anhydride"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_3-4-acetyloxyphenylglutaric_anhydride

echo ""

# 22. Synthesis_of_Acid_Chloride_using_Thionyl_Chloride
echo "📋 [22/30] 运行: Synthesis_of_Acid_Chloride_using_Thionyl_Chloride"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_Acid_Chloride_using_Thionyl_Chloride

echo ""

# 23. Synthesis_of_Ethyl_2-ethoxymethylene-3-oxobutanoat
echo "📋 [23/30] 运行: Synthesis_of_Ethyl_2-ethoxymethylene-3-oxobutanoat"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_Ethyl_2-ethoxymethylene-3-oxobutanoat

echo ""

# 24. Synthesis_of_N-2-cyclopropylphenylcarbamothioylben
echo "📋 [24/30] 运行: Synthesis_of_N-2-cyclopropylphenylcarbamothioylben"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_N-2-cyclopropylphenylcarbamothioylben

echo ""

# 25. Synthesis_of_N-Benzoyl-N-benzylthiourea
echo "📋 [25/30] 运行: Synthesis_of_N-Benzoyl-N-benzylthiourea"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_N-Benzoyl-N-benzylthiourea

echo ""

# 26. Synthesis_of_Piperidine-Hydantoin
echo "📋 [26/30] 运行: Synthesis_of_Piperidine-Hydantoin"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_Piperidine-Hydantoin

echo ""

# 27. Synthesis_of_Thiazole_Derivative_via_Hantzsch_Cycl
echo "📋 [27/30] 运行: Synthesis_of_Thiazole_Derivative_via_Hantzsch_Cycl"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_Thiazole_Derivative_via_Hantzsch_Cycl

echo ""

# 28. Synthesis_of_Thiourea_Derivative
echo "📋 [28/30] 运行: Synthesis_of_Thiourea_Derivative"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_Thiourea_Derivative

echo ""

# 29. Synthesis_of_a_Piperazinyl-Quinoline_Derivative
echo "📋 [29/30] 运行: Synthesis_of_a_Piperazinyl-Quinoline_Derivative"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Synthesis_of_a_Piperazinyl-Quinoline_Derivative

echo ""

# 30. Urea_Synthesis_via_Isocyanate_Addition
echo "📋 [30/30] 运行: Urea_Synthesis_via_Isocyanate_Addition"
python3 main.py \
    --config-dir "$CONFIG_DIR" \
    --config-name batch_level5_Navigation_Urea_Synthesis_via_Isocyanate_Addition

echo ""

echo "================================================================"
echo "✅ 所有 scene19 配置文件运行完成！"
echo "📊 总计: 30 个配置"
echo "📷 相机: Camera_03"
echo "🎬 USD: gemini_scene_19"
echo "================================================================"
