import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';

type ModuleCardProps = {
  id?: number;
  title: string;
  weeks?: string;
  description: string;
  link?: string;
  outcomes?: string[];
};

export default function ModuleCard({title, weeks, description, link = '#', outcomes}: ModuleCardProps): ReactNode {
  return (
    <div className="bg-white border border-gray-200 rounded-lg p-6 hover:shadow-md hover:border-green-600 hover:-translate-y-1 transition-all">
      {weeks && <div className="text-xs text-gray-500 font-medium uppercase mb-3">{weeks}</div>}
      <h3 className="text-xl font-bold text-green-700 mb-3" style={{color: '#059669'}}>
        {title}
      </h3>
      <p className="text-base text-gray-600 leading-relaxed mb-4">
        {description}
      </p>

      {outcomes && outcomes.length > 0 && (
        <div className="mb-4 pb-4 border-t border-gray-200">
          <h4 className="text-xs font-semibold text-gray-500 uppercase tracking-wide mb-3">Learning Outcomes</h4>
          <ul className="space-y-2">
            {outcomes.map((outcome, idx) => (
              <li key={idx} className="text-sm text-gray-700 flex items-start gap-2">
                <span className="text-green-600 font-bold mt-0.5">✓</span>
                <span>{outcome}</span>
              </li>
            ))}
          </ul>
        </div>
      )}

      <Link to={link} className="text-green-700 font-medium text-sm hover:underline" style={{color: '#059669'}}>
        Learn more →
      </Link>
    </div>
  );
}
