import { EnhancedSequenceManager } from "@/components/ros/EnhancedSequenceManager";
import { useLanguage } from "@/contexts/LanguageContext";

const Sequences = () => {
  const { t } = useLanguage();

  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">{t("sequence.title")}</h1>
        <p className="text-muted-foreground">{t("sequence.subtitle")}</p>
      </div>
      <EnhancedSequenceManager />
    </div>
  );
};

export default Sequences;
